#include "tdoa2.h"
#include "mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static uint8_t previousAnchor;
static lpsLppShortPacket_t lppPacket;
static bool lppPacketToSend = false;
static int lppPacketSendTryCounter = 0;
static bool rangingOk = false;

// Anchor configuration
static lpsTdoa2AlgoOptions_t defaultOptions = {
    .anchorAddress = {
      0xbccf000000000000,
      0xbccf000000000001,
      0xbccf000000000002,
      0xbccf000000000003,
      0xbccf000000000004,
      0xbccf000000000005,
      0xbccf000000000006,
      0xbccf000000000007,
    },
};

static double clockCorrection = 1.0;
static uint8_t clockCorrectionBucket = 0;
static tdoaAnchorInfo_t anchorInfoArray[LOCODECK_NR_OF_TDOA2_ANCHORS];

static void init(dwDevice_t *dev) {
    // uint32_t now_ms = (uint32_t) xTaskGetTickCount();
    // tdoaEngineInit(&tdoaEngineState, now_ms, sendTdoaToEstimatorCallback, LOCODECK_TS_FREQ, TdoaEngineMatchingAlgorithmYoungest);
    for (int i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
        anchorInfoArray[i].id = i;
        anchorInfoArray[i].isInitialized = true;
    }


    dwSetReceiveWaitTimeout(dev, TDOA2_RECEIVE_TIMEOUT);
    dwCommitConfiguration(dev);

    previousAnchor = 0;
    lppPacketToSend = false;
    rangingOk = false;
}

static bool isRangingOk() {
    return rangingOk;
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
}

static uint64_t _trunc(uint64_t value) {
    return value & 0x00FFFFFFFF;
}

static bool rxCallback(dwDevice_t *dev) {
    static int debug_count = 0;

    int dataLength = dwGetDataLength(dev);
    packet_t rxPacket;
  
    dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
    const rangePacket2_t* packet = (rangePacket2_t*)rxPacket.payload;
  
    bool lppSent = false;
    if (packet->type == 0x22) {
        const uint8_t anchor = rxPacket.sourceAddress & 0xff;
    
        dwTime_t arrival = {.full = 0};
        dwGetReceiveTimestamp(dev, &arrival);
        
        if (anchor < LOCODECK_NR_OF_TDOA2_ANCHORS) {
            uint32_t now_ms = (uint32_t) xTaskGetTickCount();
    
            const int64_t rxAn_by_T_in_cl_T = arrival.full;
            const int64_t txAn_in_cl_An = packet->timestamps[anchor];
            const uint8_t seqNr = packet->sequenceNrs[anchor] & 0x7f;
            
            // Get the anchor context
            tdoaAnchorContext_t anchorCtx = {0};
            for (uint8_t i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
                if (anchorInfoArray[i].id == anchor) {
                    anchorCtx.anchorInfo = &anchorInfoArray[i];
                    anchorCtx.currentTime_ms = now_ms;
                    break;
                }
            }

            if (anchorCtx.anchorInfo == NULL) {
                printf("Anchor %d not found\n", anchor);
                return false;
            }

            // Update remote anchor data
            for (uint8_t remoteId = 0; remoteId < LOCODECK_NR_OF_TDOA2_ANCHORS; remoteId++) {
                if (remoteId != anchor) {
                    int64_t remoteRxTime = packet->timestamps[remoteId];
                    uint8_t remoteSeqNr = packet->sequenceNrs[remoteId] & 0x7f;
                    
                    if (remoteRxTime != 0) {
                        anchorCtx.anchorInfo->remoteAnchorData[remoteId].id = remoteId;
                        anchorCtx.anchorInfo->remoteAnchorData[remoteId].seqNr = remoteSeqNr;
                        anchorCtx.anchorInfo->remoteAnchorData[remoteId].rxTime = remoteRxTime;
                        anchorCtx.anchorInfo->remoteAnchorData[remoteId].endOfLife = now_ms + 30;
                    }

                    uint16_t remoteDistance = packet->distances[remoteId];
                    if (remoteDistance != 0) {
                        anchorCtx.anchorInfo->remoteTof[remoteId].id = remoteId;
                        anchorCtx.anchorInfo->remoteTof[remoteId].tof = (int64_t) remoteDistance;
                        anchorCtx.anchorInfo->remoteTof[remoteId].endOfLife = now_ms + 2000;
                    }
                }
            }

            // TODO: clock correction
            const int64_t latest_rxAn_by_T_in_cl_T = anchorCtx.anchorInfo->rxTime;
            const int64_t latest_txAn_in_cl_An = anchorCtx.anchorInfo->txTime;
            if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
                const uint64_t tickCount_in_cl_reference = _trunc(rxAn_by_T_in_cl_T - latest_rxAn_by_T_in_cl_T);
                const uint64_t tickCount_in_cl_x = _trunc(txAn_in_cl_An - latest_txAn_in_cl_An);
                double clockCorrectionCandidate = -1;
                if (tickCount_in_cl_x != 0) {
                    clockCorrectionCandidate = (double)tickCount_in_cl_reference / (double)tickCount_in_cl_x;
                }

                const double difference = clockCorrectionCandidate - clockCorrection;
                if (difference < 0.03e-6 && difference > -0.03e-6) {
                    clockCorrection = clockCorrection * 0.9 + clockCorrectionCandidate * 0.1;
                    if (clockCorrectionBucket < 4)
                        clockCorrectionBucket++;
                } else {
                    if (clockCorrectionBucket > 0)
                        clockCorrectionBucket--;
                    else if (1 - 20e-6 < clockCorrectionCandidate && clockCorrectionCandidate < 1 + 20e-6) {
                        clockCorrection = clockCorrectionCandidate;
                    }
                }
            }

            // Process the packet, find the youngest anchor
            tdoaAnchorContext_t otherAnchorCtx = {0};
            uint8_t candidate[LOCODECK_NR_OF_TDOA2_ANCHORS] = {0};

            for (uint8_t i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
                if (anchorCtx.anchorInfo->remoteAnchorData[i].endOfLife > now_ms) {
                    candidate[i] = 1;
                }
            }
            int youngestAnchorId = -1;
            uint32_t youngestUpdateTime = 0;
            for (uint8_t i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
                if (candidate[i] && anchorCtx.anchorInfo->remoteTof[i].endOfLife > now_ms && anchorCtx.anchorInfo->remoteTof[i].tof != 0) {
                    otherAnchorCtx.anchorInfo = &anchorInfoArray[i];
                    if (otherAnchorCtx.anchorInfo->lastUpdateTime > youngestUpdateTime
                        && otherAnchorCtx.anchorInfo->seqNr == anchorCtx.anchorInfo->remoteAnchorData[i].seqNr) {
                        youngestUpdateTime = otherAnchorCtx.anchorInfo->lastUpdateTime;
                        youngestAnchorId = i;
                    }
                }
            }

            if (youngestAnchorId != -1) {
                otherAnchorCtx.anchorInfo = &anchorInfoArray[youngestAnchorId];
                otherAnchorCtx.currentTime_ms = now_ms;

                // Calculate the time of flight
                const int64_t tof_Ar_to_An_in_cl_An = anchorCtx.anchorInfo->remoteTof[youngestAnchorId].tof;
                const int64_t rxAr_by_An_in_cl_An = anchorCtx.anchorInfo->remoteAnchorData[youngestAnchorId].rxTime;
                const int64_t rxAr_by_T_in_cl_T = otherAnchorCtx.anchorInfo->rxTime;
                const int64_t delta_txAr_to_txAn_in_cl_An = tof_Ar_to_An_in_cl_An + _trunc(txAn_in_cl_An - rxAr_by_An_in_cl_An);
                const double timeDiffOfArrival_in_cl_T = (double)_trunc(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - clockCorrection * (double)delta_txAr_to_txAn_in_cl_An;
                
                const double distance = (double) timeDiffOfArrival_in_cl_T * SPEED_OF_LIGHT / (double)LOCODECK_TS_FREQ;

                if (debug_count % 50 == 0 || debug_count % 50 == 1) {
                    printf("Distance to anchor %d: %.2f\n", youngestAnchorId, distance);
                    
                }
                debug_count++;
            }

            // Set the anchor status
            anchorCtx.anchorInfo->lastUpdateTime = now_ms;
            anchorCtx.anchorInfo->txTime = txAn_in_cl_An;
            anchorCtx.anchorInfo->rxTime = rxAn_by_T_in_cl_T;
            anchorCtx.anchorInfo->seqNr = seqNr;
    
            rangingOk = true;
        }
    }

    return lppSent;
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
    switch(event) {
        case eventPacketReceived:
            if (rxCallback(dev)) {
                lppPacketToSend = false;
            } else {
                setRadioInReceiveMode(dev);
    
                // Discard lpp packet if we cannot send it for too long
                if (++lppPacketSendTryCounter >= TDOA2_LPP_PACKET_SEND_TIMEOUT) {
                    lppPacketToSend = false;
                }
            }
    
            // if (!lppPacketToSend) {
            //     // Get next lpp packet
            //     lppPacketToSend = lpsGetLppShort(&lppPacket);
            //     lppPacketSendTryCounter = 0;
            // }
            break;
        case eventTimeout:
            // Fall through
        case eventReceiveFailed:
            // Fall through
        case eventReceiveTimeout:
            setRadioInReceiveMode(dev);
            break;
        case eventPacketSent:
            // Service packet sent, the radio is back to receive automatically
            break;
        default:
            printf("TDOA2: UNEXPECTED EVENT\n");
            break;
    }
  
    return portMAX_DELAY;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
    // Example: Retrieve from a predefined array (replace with actual data)
    const point_t anchorPositions[] = {
        {x: 0.0, y: 0.0, z: 0.0},   // Anchor 0
        {x: 1.0, y: 0.0, z: 0.0},   // Anchor 1
        {x: 1.0, y: 0.0, z: 0.0},   // Anchor 2
        {x: 1.0, y: 0.0, z: 0.0},   // Anchor 3
        {x: 1.0, y: 0.0, z: 0.0},   // Anchor 4
        {x: 1.0, y: 0.0, z: 0.0},   // Anchor 5
        {x: 1.0, y: 0.0, z: 0.0},   // Anchor 6
        {x: 1.0, y: 0.0, z: 0.0},   // Anchor 7
        // ... Add other anchor positions
    };
    if (anchorId < LOCODECK_NR_OF_TDOA2_ANCHORS) {
        *position = anchorPositions[anchorId];
        return true;
    }
    return false;
}
  
static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    // return tdoaStorageGetListOfAnchorIds(tdoaEngineState.anchorInfoArray, unorderedAnchorList, maxListSize);
    return 0;
}
  
static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
    // uint32_t now_ms = (uint32_t) xTaskGetTickCount();
    // return tdoaStorageGetListOfActiveAnchorIds(tdoaEngineState.anchorInfoArray, unorderedAnchorList, maxListSize, now_ms);}
    return 0;
}

uwbAlgorithm_t uwbTdoa2TagAlgorithm = {
    .init = init,
    .onEvent = onEvent,
    .isRangingOk = isRangingOk,
    .getAnchorPosition = getAnchorPosition,
    .getAnchorIdList = getAnchorIdList,
    .getActiveAnchorIdList = getActiveAnchorIdList,
  };
#include "tdoa2.h"
#include "mac.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "estimator.h"

static tdoaAnchorInfo_t anchorInfoArray[LOCODECK_NR_OF_TDOA2_ANCHORS];

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
    // Example: Retrieve from a predefined array (replace with actual data)
    const point_t anchorPositions[] = {
        {x: 0.0, y: 0.0, z: 0.0},   // Anchor 0
        {x: 2.0, y: 0.0, z: 0.0},   // Anchor 1
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

static void init(dwDevice_t *dev) {
    for (int i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
        anchorInfoArray[i].id = i;
        anchorInfoArray[i].isInitialized = true;
        getAnchorPosition(i, &anchorInfoArray[i].position);
    }

    dwSetReceiveWaitTimeout(dev, TDOA2_RECEIVE_TIMEOUT);
    dwCommitConfiguration(dev);
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
}

static uint64_t _trunc(uint64_t value) {
    return value & 0x00FFFFFFFF;
}

static void rxCallback(dwDevice_t *dev) {
    static int debug_count = 0;

    int dataLength = dwGetDataLength(dev);
    packet_t rxPacket;
  
    dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
    const rangePacket2_t* packet = (rangePacket2_t*)rxPacket.payload;

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
                return;
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
            bool sampleIsReliable = false;
            const int64_t latest_rxAn_by_T_in_cl_T = anchorCtx.anchorInfo->rxTime;
            const int64_t latest_txAn_in_cl_An = anchorCtx.anchorInfo->txTime;
            if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
                const uint64_t tickCount_in_cl_reference = _trunc(rxAn_by_T_in_cl_T - latest_rxAn_by_T_in_cl_T);
                const uint64_t tickCount_in_cl_x = _trunc(txAn_in_cl_An - latest_txAn_in_cl_An);
                double clockCorrectionCandidate = -1;
                if (tickCount_in_cl_x != 0) {
                    clockCorrectionCandidate = (double)tickCount_in_cl_reference / (double)tickCount_in_cl_x;
                }

                double currentClockCorrection = anchorCtx.anchorInfo->clockCorrection;
                const double difference = clockCorrectionCandidate - currentClockCorrection;
                if (difference < 0.03e-6 && difference > -0.03e-6) {
                    anchorCtx.anchorInfo->clockCorrection = currentClockCorrection * 0.1 + clockCorrectionCandidate * 0.9;
                    if (anchorCtx.anchorInfo->clockCorrectionBucket < 4)
                        anchorCtx.anchorInfo->clockCorrectionBucket++;
                    sampleIsReliable = true;
                } else {
                    if (anchorCtx.anchorInfo->clockCorrectionBucket > 0)
                        anchorCtx.anchorInfo->clockCorrectionBucket--;
                    else if (1 - 20e-6 < clockCorrectionCandidate && clockCorrectionCandidate < 1 + 20e-6) {
                        anchorCtx.anchorInfo->clockCorrection = clockCorrectionCandidate;
                    }
                }
            }

            if (sampleIsReliable) {
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
                    const double timeDiffOfArrival_in_cl_T = (double)_trunc(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - anchorCtx.anchorInfo->clockCorrection * (double)delta_txAr_to_txAn_in_cl_An;
                    
                    const float distance = (float)((double) timeDiffOfArrival_in_cl_T * SPEED_OF_LIGHT / (double)LOCODECK_TS_FREQ);
                    
                    tdoaRemoteAnchorData_t *ra = &anchorCtx.anchorInfo->remoteAnchorData[youngestAnchorId];
                    float mean = ra->historySum / TDOA2_REMOTE_HISTORY_COUNT;
                    ra->historySum += distance - ra->history[ra->historyIndex];
                    ra->history[ra->historyIndex] = distance;
                    ra->historyIndex = (ra->historyIndex + 1) % TDOA2_REMOTE_HISTORY_COUNT;
                    if (fabsf(mean - distance) < 1.0f) {
                        // if (debug_count % 50 == 0 || debug_count % 50 == 1) {
                        // printf("%d -> %d: %.2f %f\n", anchor, youngestAnchorId, distance);
                        // }
                        estimatorPacket_t packet;
                        packet.type = ESTIMATOR_TYPE_UWB;
                        packet.tdoa.distanceDiff = distance;
                        packet.tdoa.stdDev = 0.15f;

                        packet.tdoa.anchorIds[0] = anchorCtx.anchorInfo->id;
                        packet.tdoa.anchorIds[1] = otherAnchorCtx.anchorInfo->id;
                        packet.tdoa.anchorPositions[0] = anchorCtx.anchorInfo->position;
                        packet.tdoa.anchorPositions[1] = otherAnchorCtx.anchorInfo->position;
                        estimatorKalmanEnqueue(&packet);
                    }

                    debug_count++;
                }
            }

            // Set the anchor status
            anchorCtx.anchorInfo->lastUpdateTime = now_ms;
            anchorCtx.anchorInfo->txTime = txAn_in_cl_An;
            anchorCtx.anchorInfo->rxTime = rxAn_by_T_in_cl_T;
            anchorCtx.anchorInfo->seqNr = seqNr;
        }
    }
}

static void onEvent(dwDevice_t *dev, uwbEvent_t event) {
    switch (event) {
        case eventPacketReceived:
            rxCallback(dev);
            setRadioInReceiveMode(dev);
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
}

uwbAlgorithm_t uwbTdoa2TagAlgorithm = {
    .init = init,
    .onEvent = onEvent,
    .getAnchorPosition = getAnchorPosition,
  };
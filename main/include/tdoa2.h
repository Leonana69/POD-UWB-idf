#ifndef __TDOA2_H__
#define __TDOA2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "loco.h"

#define TDOA2_RECEIVE_TIMEOUT 10000
#define TDOA2_LPP_PACKET_SEND_TIMEOUT (LOCODECK_NR_OF_TDOA2_ANCHORS * 5)

typedef struct {
    uint8_t type;
    uint8_t sequenceNrs[LOCODECK_NR_OF_TDOA2_ANCHORS];
    uint32_t timestamps[LOCODECK_NR_OF_TDOA2_ANCHORS];
    uint16_t distances[LOCODECK_NR_OF_TDOA2_ANCHORS];
} __attribute__((packed)) rangePacket2_t;

typedef struct {
    uint8_t id; // Id of remote remote anchor
    uint8_t seqNr; // Sequence number of the packet received in the remote anchor (7 bits)
    int64_t rxTime; // Receive time of packet from anchor id in the remote anchor, in remote DWM clock
    uint32_t endOfLife;
} tdoaRemoteAnchorData_t;
  
typedef struct {
    uint8_t id;
    int64_t tof;
    uint32_t endOfLife; // Time stamp when the tof data is outdated, local system time in ms
} tdoaTimeOfFlight_t;

typedef struct {
    bool isInitialized;
    uint32_t lastUpdateTime; // The time when this anchor was updated the last time
    uint8_t id; // Anchor id
  
    int64_t txTime; // Transmit time of last packet, in remote DWM clock
    int64_t rxTime; // Receive time of last packet, in local DWM clock
    uint8_t seqNr; // Sequence nr of last packet (7 bits)
  
    // clockCorrectionStorage_t clockCorrectionStorage;
  
    point_t position; // The coordinates of the anchor
  
    tdoaTimeOfFlight_t remoteTof[LOCODECK_NR_OF_TDOA2_ANCHORS];
    tdoaRemoteAnchorData_t remoteAnchorData[LOCODECK_NR_OF_TDOA2_ANCHORS];
} tdoaAnchorInfo_t;

typedef struct {
    tdoaAnchorInfo_t *anchorInfo;
    uint32_t currentTime_ms;
} tdoaAnchorContext_t;

extern uwbAlgorithm_t uwbTdoa2TagAlgorithm;

#ifdef __cplusplus
}
#endif
#endif
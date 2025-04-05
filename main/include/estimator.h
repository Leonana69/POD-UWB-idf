#ifndef __ESTIMATOR_H__
#define __ESTIMATOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

void estimatorInit();
void estimatorKalmanEnqueue(estimatorPacket_t *packet);
void estimatorKalmanGetState(state_t *state);

#ifdef __cplusplus
}
#endif

#endif
#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "types.h"

void imuInit();
vec3f_t getInitialAccel();

#ifdef __cplusplus
}
#endif

#endif
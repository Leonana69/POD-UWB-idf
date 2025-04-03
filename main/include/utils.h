#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

float degrees(float radians);
float radians(float degrees);

#define ABS(x) ((x) > 0 ? (x) : -(x))

#ifdef __cplusplus
}
#endif

#endif // __UTILS_H__
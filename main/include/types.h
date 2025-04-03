#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>
#include <stdbool.h>

typedef float scalar_t;

typedef union {
    struct {
        scalar_t x;
        scalar_t y;
        scalar_t z;
    };
    scalar_t v[3];
}__attribute__((packed, aligned(4))) vec3f_t;

typedef struct {
    vec3f_t accel;             // Gs
    vec3f_t gyro;              // deg/s
} imu_t;

#endif
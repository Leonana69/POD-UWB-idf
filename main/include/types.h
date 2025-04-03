#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>
#include <stdbool.h>

typedef float scalar_t;
/** Data structure used by the stabilizer subsystem.
 *  All have a timestamp to be set when the data is calculated.
 */
struct geo_info_s {
	union {
		struct {
			scalar_t roll;
			scalar_t pitch;
			scalar_t yaw;
		};
		struct {
			scalar_t x;
			scalar_t y;
            scalar_t z;
		};
	};
}__attribute__((packed, aligned(4)));

typedef struct geo_info_s attitude_t;
typedef struct geo_info_s palstance_t;
typedef struct geo_info_s position_t;
typedef struct geo_info_s velocity_t;
typedef struct geo_info_s accel_t;

/* Orientation as a quaternion */
typedef struct {
    union {
        struct {
            scalar_t q0;
            scalar_t q1;
            scalar_t q2;
            scalar_t q3;
        };
        struct {
            scalar_t x;
            scalar_t y;
            scalar_t z;
            scalar_t w;
        };
    };
}__attribute__((packed, aligned(4))) quaternion_t;

typedef union {
    struct {
        scalar_t x;
        scalar_t y;
        scalar_t z;
    };
    scalar_t v[3];
}__attribute__((packed, aligned(4))) vec3f_t;

typedef struct {
    attitude_t attitude;
    quaternion_t attitudeQuat;
    position_t position;      // m
    velocity_t velocity;      // m/s
    accel_t accel;            // Gs (but acc.z without considering gravity)
} state_t;

typedef struct {
    uint32_t timestamp;
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} state_com_t;

typedef struct {
    attitude_t attitude;
    scalar_t thrust;
} control_t;

typedef enum {
    STABILIZE_DISABLE = 0,
    STABILIZE_ABSOLUTE = 1,
    STABILIZE_VELOCITY = 2,
} stab_mode_t;

typedef struct {
    uint32_t timestamp;
    uint32_t duration;

    scalar_t thrust;
    union {
        attitude_t attitude;   // deg
        palstance_t palstance; // deg/s
    };
    union {
        position_t position;  // m
        velocity_t velocity;  // m/s
    };
    bool velocity_body;     // true if velocity is given in body frame

    struct {
        uint8_t x;
        uint8_t y;
        uint8_t z;
        uint8_t roll;
        uint8_t pitch;
        uint8_t yaw;
    } mode;
} setpoint_t;

/*
 * Data structure used by all sensors
 */
typedef struct {
    scalar_t pressure;           // mbar
    scalar_t temperature;        // degree Celcius
    scalar_t asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct {
    scalar_t distance;          // m
    scalar_t stdDev;            // m
    scalar_t dt;
} tof_t;

typedef struct {
    union {
        struct {
            scalar_t dpixelx;  // Accumulated pixel count x
            scalar_t dpixely;  // Accumulated pixel count y
        };
        scalar_t dpixel[2];  // Accumulated pixel count
    };
    scalar_t stdDevX;      // Measurement standard deviation
    scalar_t stdDevY;      // Measurement standard deviation
    scalar_t dt;           // Time during which pixels were accumulated
} flow_t;

typedef struct {
    scalar_t dx;
    scalar_t dy;
    scalar_t stdDevX;
    scalar_t stdDevY;
    scalar_t dt;
} motor_t;

typedef struct {
    vec3f_t accel;             // Gs
    vec3f_t gyro;              // deg/s
} imu_t;

// Frequencies to bo used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define DT_1000_HZ 0.001
#define DT_500_HZ 0.002
#define DT_100_HZ 0.01
#define DT_50_HZ 0.02
#define DT_25_HZ 0.04

#define RATE_MAIN_LOOP 			RATE_1000_HZ
#define ATTITUDE_RATE 			RATE_500_HZ
#define ATTITUDE_UPDATE_DT 	    DT_500_HZ
#define POSITION_RATE 			RATE_100_HZ
#define POSITION_UPDATE_DT 	    DT_100_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

#endif
#include "kalman_core.h"
#include <string.h>
#include <math.h>
#include "utils.h"

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100.0f)
#define MIN_COVARIANCE (1e-6f)
#define EPSILON        (1e-6f)
#define ROLLPITCH_ZERO_REVERSION (0.001f)

// Initial variances, uncertain of position, but know we're stationary
static const float initStdDevPos_xy = 100;
static const float initStdDevPos_z = 1;
static const float initStdDevVel_xyz = 0.01;
static const float initStdDevAtt_rpy = 0.01;

static float procNoiseAcc_xy = 0.5f;
static float procNoiseAcc_z = 0.5f;
static float procNoiseVel = 0.1f;       // ← increased
static float procNoisePos = 0.01f;      // ← small but nonzero
static float procNoiseAtt = 0.001f;     // ← attitude drift
static float measNoiseGyr_rpy = 0.1f;
// static float measNoiseGyr_rpy = 0.000041f;  // in rad/s
// static float measNoiseAccel_xy       = 0.00047f;   // in m/s²
// static float measNoiseAccel_z        = 0.00047f;   // in m/s²

Kalman::Kalman(vec3f_t initialAccel) {
    memset(S, 0, sizeof(S));
    memset(q, 0, sizeof(q));
    memset(R, 0, sizeof(R));
    memset(P, 0, sizeof(P));

    S[KC_STATE_X] = 1.0f; // x
    S[KC_STATE_Y] = 1.0f; // y
    S[KC_STATE_Z] = 1.0f; // z

    // Initialize the rotation matrix and quaternion
    float ax = initialAccel.x;
    float ay = initialAccel.y;
    float az = initialAccel.z;

    float norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;
    // Compute roll and pitch from gravity
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float yaw = 0.0f; // No way to estimate yaw with just accel

    // Convert to quaternion
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    _UpdateRotationMatrix();

    // Initialize the covariance matrix
    P[KC_STATE_X][KC_STATE_X] = powf(initStdDevPos_xy, 2);
    P[KC_STATE_Y][KC_STATE_Y] = powf(initStdDevPos_xy, 2);
    P[KC_STATE_Z][KC_STATE_Z] = powf(initStdDevPos_z, 2);
    P[KC_STATE_PX][KC_STATE_PX] = powf(initStdDevVel_xyz, 2);
    P[KC_STATE_PY][KC_STATE_PY] = powf(initStdDevVel_xyz, 2);
    P[KC_STATE_PZ][KC_STATE_PZ] = powf(initStdDevVel_xyz, 2);
    P[KC_STATE_D0][KC_STATE_D0] = powf(initStdDevAtt_rpy, 2);
    P[KC_STATE_D1][KC_STATE_D1] = powf(initStdDevAtt_rpy, 2);
    P[KC_STATE_D2][KC_STATE_D2] = powf(initStdDevAtt_rpy, 2);
    Pm = dspm::Mat((float *)P, KC_STATE_DIM, KC_STATE_DIM);
}

Kalman::~Kalman() {
    // optionally empty
}

void Kalman::Predict(imu_t* imuData, float dt, bool isFlying) {
    // The state transition matrix
    dspm::Mat Fm = dspm::Mat::eye(KC_STATE_DIM);

    vec3f_t *gyro = &imuData->gyro;
    vec3f_t *accel = &imuData->accel;

    // position from body-frame velocity
    Fm(KC_STATE_X, KC_STATE_PX) = R[0][0] * dt;
    Fm(KC_STATE_Y, KC_STATE_PX) = R[1][0] * dt;
    Fm(KC_STATE_Z, KC_STATE_PX) = R[2][0] * dt;

    Fm(KC_STATE_X, KC_STATE_PY) = R[0][1] * dt;
    Fm(KC_STATE_Y, KC_STATE_PY) = R[1][1] * dt;
    Fm(KC_STATE_Z, KC_STATE_PY) = R[2][1] * dt;

    Fm(KC_STATE_X, KC_STATE_PZ) = R[0][2] * dt;
    Fm(KC_STATE_Y, KC_STATE_PZ) = R[1][2] * dt;
    Fm(KC_STATE_Z, KC_STATE_PZ) = R[2][2] * dt;

    // position from attitude error
    Fm(KC_STATE_X, KC_STATE_D0) = (S[KC_STATE_PY] * R[0][2] - S[KC_STATE_PZ] * R[0][1]) * dt;
    Fm(KC_STATE_Y, KC_STATE_D0) = (S[KC_STATE_PY] * R[1][2] - S[KC_STATE_PZ] * R[1][1]) * dt;
    Fm(KC_STATE_Z, KC_STATE_D0) = (S[KC_STATE_PY] * R[2][2] - S[KC_STATE_PZ] * R[2][1]) * dt;

    Fm(KC_STATE_X, KC_STATE_D1) = (-S[KC_STATE_PX] * R[0][2] + S[KC_STATE_PZ] * R[0][0]) * dt;
    Fm(KC_STATE_Y, KC_STATE_D1) = (-S[KC_STATE_PX] * R[1][2] + S[KC_STATE_PZ] * R[1][0]) * dt;
    Fm(KC_STATE_Z, KC_STATE_D1) = (-S[KC_STATE_PX] * R[2][2] + S[KC_STATE_PZ] * R[2][0]) * dt;

    Fm(KC_STATE_X, KC_STATE_D2) = (S[KC_STATE_PX] * R[0][1] - S[KC_STATE_PY] * R[0][0]) * dt;
    Fm(KC_STATE_Y, KC_STATE_D2) = (S[KC_STATE_PX] * R[1][1] - S[KC_STATE_PY] * R[1][0]) * dt;
    Fm(KC_STATE_Z, KC_STATE_D2) = (S[KC_STATE_PX] * R[2][1] - S[KC_STATE_PY] * R[2][0]) * dt;

    // body-frame velocity change from attitude change (rotation)
    Fm(KC_STATE_PX, KC_STATE_PX) = 1.0;
    Fm(KC_STATE_PY, KC_STATE_PX) = -gyro->z * dt;
    Fm(KC_STATE_PZ, KC_STATE_PX) = gyro->y * dt;

    Fm(KC_STATE_PX, KC_STATE_PY) = gyro->z * dt;
    Fm(KC_STATE_PY, KC_STATE_PY) = 1.0;
    Fm(KC_STATE_PZ, KC_STATE_PY) = -gyro->x * dt;

    Fm(KC_STATE_PX, KC_STATE_PZ) = -gyro->y * dt;
    Fm(KC_STATE_PY, KC_STATE_PZ) = gyro->x * dt;
    Fm(KC_STATE_PZ, KC_STATE_PZ) = 1.0;

    // body-frame velocity from attitude error
    Fm(KC_STATE_PX, KC_STATE_D0) = 0.0;
    // delta_V_PY = -g_PZ * sin(delta_roll) * dt = -g_PZ * delta_roll * dt
    Fm(KC_STATE_PY, KC_STATE_D0) = -GRAVITY_EARTH * R[2][2] * dt;
    Fm(KC_STATE_PZ, KC_STATE_D0) = GRAVITY_EARTH * R[2][1] * dt;

    Fm(KC_STATE_PX, KC_STATE_D1) = GRAVITY_EARTH * R[2][2] * dt;
    Fm(KC_STATE_PY, KC_STATE_D1) = 0.0;
    Fm(KC_STATE_PZ, KC_STATE_D1) = -GRAVITY_EARTH * R[2][0] * dt;

    Fm(KC_STATE_PX, KC_STATE_D2) = -GRAVITY_EARTH * R[2][1] * dt;
    Fm(KC_STATE_PY, KC_STATE_D2) = GRAVITY_EARTH * R[2][0] * dt;
    Fm(KC_STATE_PZ, KC_STATE_D2) = 0.0;

    // attitude error from attitude error
    /**
     * At first glance, it may not be clear where the next values come from, since they do not appear directly in the
     * dynamics. In this prediction step, we skip the step of first updating attitude-error, and then incorporating the
     * new error into the current attitude (which requires a rotation of the attitude-error covariance). Instead, we
     * directly update the body attitude, however still need to rotate the covariance, which is what you see below.
     *
     * This comes from a second order approximation to:
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * where d is the attitude error expressed as Rodriges parameters, ie. d0 = 1/2*gyro.x*dt under the assumption that
     * d = [0,0,0] at the beginning of each prediction step and that gyro.x is constant over the sampling period
     *
     * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */
    float d0 = gyro->x * dt / 2;
    float d1 = gyro->y * dt / 2;
    float d2 = gyro->z * dt / 2;

    Fm(KC_STATE_D0, KC_STATE_D0) = 1 - d1 * d1 / 2 - d2 * d2 / 2;
    Fm(KC_STATE_D0, KC_STATE_D1) = d2 + d0 * d1 / 2;
    Fm(KC_STATE_D0, KC_STATE_D2) = -d1 + d0 * d2 / 2;

    Fm(KC_STATE_D1, KC_STATE_D0) = -d2 + d0 * d1 / 2;
    Fm(KC_STATE_D1, KC_STATE_D1) = 1 - d0 * d0 / 2 - d2 * d2 / 2;
    Fm(KC_STATE_D1, KC_STATE_D2) = d0 + d1 * d2 / 2;

    Fm(KC_STATE_D2, KC_STATE_D0) = d1 + d0 * d2 / 2;
    Fm(KC_STATE_D2, KC_STATE_D1) = -d0 + d1 * d2 / 2;
    Fm(KC_STATE_D2, KC_STATE_D2) = 1 - d0 * d0 / 2 - d1 * d1 / 2;

    // ====== COVARIANCE UPDATE: P_n+1,n = F * P_n,n * F^T + Q ======
    // F * P_n,n
    dspm::Mat tmpNN1m = Fm * Pm;
    // F^T
    dspm::Mat tmpNN2m = Fm.t();
    // F * P_n,n * F^T
    Pm = tmpNN1m * tmpNN2m;

    // ====== PREDICTION STEP: S_n+1,n = F * S_n,n + G * u_n + w_n ======
    // The control input u_n depends on whether we're on the ground, or in flight.
    // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)
    float dx, dy, dz;
    float tmpSPX, tmpSPY, tmpSPZ;
    float dt2_2 = dt * dt / 2.0f;

    if (isFlying) {
        // only acceleration in z direction
        // Use accelerometer and not commanded thrust, as this has proper physical units

        // position updates in the body frame (will be rotated to inertial frame)
        dx = S[KC_STATE_PX] * dt;
        dy = S[KC_STATE_PY] * dt;
        dz = S[KC_STATE_PZ] * dt + accel->z * dt2_2; // thrust can only be produced in the body's Z direction

        // position update
        S[KC_STATE_X] += R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
        S[KC_STATE_Y] += R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
        S[KC_STATE_Z] += R[2][0] * dx + R[2][1] * dy + R[2][2] * dz - GRAVITY_EARTH * dt2_2;

        // keep previous time step's state for the update
        tmpSPX = S[KC_STATE_PX];
        tmpSPY = S[KC_STATE_PY];
        tmpSPZ = S[KC_STATE_PZ];

        // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
        S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_EARTH * R[2][0]);
        S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_EARTH * R[2][1]);
        S[KC_STATE_PZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_EARTH * R[2][2]);
    } else {
        // Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
        // position updates in the body frame (will be rotated to inertial frame)
        dx = S[KC_STATE_PX] * dt + accel->x * dt2_2;
        dy = S[KC_STATE_PY] * dt + accel->y * dt2_2;
        dz = S[KC_STATE_PZ] * dt + accel->z * dt2_2; // thrust can only be produced in the body's Z direction

        // position update
        S[KC_STATE_X] += R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
        S[KC_STATE_Y] += R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
        S[KC_STATE_Z] += R[2][0] * dx + R[2][1] * dy + R[2][2] * dz - GRAVITY_EARTH * dt2_2;

        // keep previous time step's state for the update
        tmpSPX = S[KC_STATE_PX];
        tmpSPY = S[KC_STATE_PY];
        tmpSPZ = S[KC_STATE_PZ];

        // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
        S[KC_STATE_PX] += dt * (accel->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_EARTH * R[2][0]);
        S[KC_STATE_PY] += dt * (accel->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_EARTH * R[2][1]);
        S[KC_STATE_PZ] += dt * (accel->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_EARTH * R[2][2]);
    }

    // attitude update (rotate by gyroscope), we do this in quaternions
    // compute the quaternion values in [w,x,y,z] order
    _DeltaGyro2Quaternion(dt * gyro->x, dt * gyro->y, dt * gyro->z);
}

void Kalman::AddProcessNoise(float dt) {
    if (dt > 0) {
        // Add process noise on position
        P[KC_STATE_X][KC_STATE_X] += powf(procNoiseAcc_xy * dt * dt, 2) 
                                             + powf(procNoiseVel * dt, 2) 
                                             + powf(procNoisePos, 2);
        P[KC_STATE_Y][KC_STATE_Y] += powf(procNoiseAcc_xy * dt * dt, 2) 
                                             + powf(procNoiseVel * dt, 2) 
                                             + powf(procNoisePos, 2);
        P[KC_STATE_Z][KC_STATE_Z] += powf(procNoiseAcc_z * dt * dt, 2) 
                                             + powf(procNoiseVel * dt, 2) 
                                             + powf(procNoisePos, 2);

        // Add process noise on velocity
        P[KC_STATE_PX][KC_STATE_PX] += powf(procNoiseAcc_xy * dt, 2) 
                                               + powf(procNoiseVel, 2);
        P[KC_STATE_PY][KC_STATE_PY] += powf(procNoiseAcc_xy * dt, 2) 
                                               + powf(procNoiseVel, 2);
        P[KC_STATE_PZ][KC_STATE_PZ] += powf(procNoiseAcc_z * dt, 2) 
                                               + powf(procNoiseVel, 2);

        // Add process noise on attitude
        P[KC_STATE_D0][KC_STATE_D0] += powf(measNoiseGyr_rpy * dt, 2) 
                                               + powf(procNoiseAtt, 2);
        P[KC_STATE_D1][KC_STATE_D1] += powf(measNoiseGyr_rpy * dt, 2) 
                                               + powf(procNoiseAtt, 2);
        P[KC_STATE_D2][KC_STATE_D2] += powf(measNoiseGyr_rpy * dt, 2) 
                                               + powf(procNoiseAtt, 2);
    }

    // Cap covariance values to ensure numerical stability
    _CapCovariance();
}

void Kalman::Finalize() {
    // Matrix to rotate the attitude covariances once updated
    dspm::Mat Fm = dspm::Mat::eye(KC_STATE_DIM);

    // Incorporate the attitude error (Kalman filter state) with the attitude
    float v0 = S[KC_STATE_D0];
    float v1 = S[KC_STATE_D1];
    float v2 = S[KC_STATE_D2];

    // Move attitude error into attitude if any of the angle errors are large enough
    if ((fabsf(v0) > 1e-4f || fabsf(v1) > 1e-4f || fabsf(v2) > 1e-4f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10)) {
        _DeltaGyro2Quaternion(v0, v1, v2);

        /** Rotate the covariance, since we've rotated the body
        *
        * This comes from a second order approximation to:
        * Sigma_post = exps(-d) Sigma_pre exps(-d)'
        *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
        * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
        *
        * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
        * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
        */

        float d0 = v0 / 2; // the attitude error vector (v0,v1,v2) is small,
        float d1 = v1 / 2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
        float d2 = v2 / 2;

        Fm(KC_STATE_D0, KC_STATE_D0) = 1 - d1 * d1 / 2 - d2 * d2 / 2;
        Fm(KC_STATE_D0, KC_STATE_D1) = d2 + d0 * d1 / 2;
        Fm(KC_STATE_D0, KC_STATE_D2) = -d1 + d0 * d2 / 2;

        Fm(KC_STATE_D1, KC_STATE_D0) = -d2 + d0 * d1 / 2;
        Fm(KC_STATE_D1, KC_STATE_D1) = 1 - d0 * d0 / 2 - d2 * d2 / 2;
        Fm(KC_STATE_D1, KC_STATE_D2) = d0 + d1 * d2 / 2;

        Fm(KC_STATE_D2, KC_STATE_D0) = d1 + d0 * d2 / 2;
        Fm(KC_STATE_D2, KC_STATE_D1) = -d0 + d1 * d2 / 2;
        Fm(KC_STATE_D2, KC_STATE_D2) = 1 - d0 * d0 / 2 - d1 * d1 / 2;

        // arm_mat_trans_f32(&Fm, &tmpNN1m); // F'
        // arm_mat_mult_f32(&Fm, &Pm, &tmpNN2m); // FP
        // arm_mat_mult_f32(&tmpNN2m, &tmpNN1m, &Pm); // FPF'
        dspm::Mat tmpNN1m = Fm.t();
        dspm::Mat tmpNN2m = Fm * Pm;
        Pm = tmpNN2m * tmpNN1m;
    }

    _UpdateRotationMatrix();

    // reset the attitude error
    S[KC_STATE_D0] = 0;
    S[KC_STATE_D1] = 0;
    S[KC_STATE_D2] = 0;

    // enforce symmetry of the covariance matrix, and ensure the values stay bounded
    _CapCovariance();
}

void Kalman::_DeltaGyro2Quaternion(float d0, float d1, float d2) {
    float angle = sqrtf(d0 * d0 + d1 * d1 + d2 * d2);
    float ca = cosf(angle / 2.0f);
    float sa_angle;
    if (angle < EPSILON) {
        sa_angle = 1.0f;
    } else {
        sa_angle = sinf(angle / 2.0f) / angle;
    }
    float dq[4] = {ca, sa_angle * d0, sa_angle * d1, sa_angle * d2};

    // rotate the quad's attitude by the delta quaternion vector computed above
    float tmpQ0 = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
    float tmpQ1 = dq[1] * q[0] + dq[0] * q[1] + dq[3] * q[2] - dq[2] * q[3];
    float tmpQ2 = dq[2] * q[0] - dq[3] * q[1] + dq[0] * q[2] + dq[1] * q[3];
    float tmpQ3 = dq[3] * q[0] + dq[2] * q[1] - dq[1] * q[2] + dq[0] * q[3];

    /* This reversion would cause yaw estimation diminish to zero */
    // if (!isFlying) {
    //     float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;
    
    //     // Extract yaw from the current quaternion
    //     float yaw = atan2f(2.0f * (tmpq0 * tmpq3 + tmpq1 * tmpq2),
    //                        1.0f - 2.0f * (tmpq2 * tmpq2 + tmpq3 * tmpq3));
    
    //     // Reset roll and pitch while preserving yaw
    //     tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * cosf(yaw / 2.0f);
    //     tmpq1 = keep * tmpq1;
    //     tmpq2 = keep * tmpq2;
    //     tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * sinf(yaw / 2.0f);
    // }

    // normalize and store the result
    float norm = sqrtf(tmpQ0 * tmpQ0 + tmpQ1 * tmpQ1 + tmpQ2 * tmpQ2 + tmpQ3 * tmpQ3);
    q[0] = tmpQ0 / norm;
    q[1] = tmpQ1 / norm;
    q[2] = tmpQ2 / norm;
    q[3] = tmpQ3 / norm;
}

void Kalman::_UpdateRotationMatrix() {
    R[0][0] = 1.0 - 2 * (q[2] * q[2] + q[3] * q[3]);
    R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];

    R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
    R[1][1] = 1.0 - 2 * (q[1] * q[1] + q[3] * q[3]);
    R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];

    R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
    R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
    R[2][2] = 1.0 - 2 * (q[1] * q[1] + q[2] * q[2]);
}

void Kalman::_CapCovariance() {
    for (int i = 0; i < KC_STATE_DIM; i++) {
        for (int j = i; j < KC_STATE_DIM; j++) {
            float p = 0.5f * P[i][j] + 0.5f * P[j][i];
            if (isnan(p) || p > MAX_COVARIANCE)
                P[i][j] = P[j][i] = MAX_COVARIANCE;
            else if (i == j && p < MIN_COVARIANCE)
                P[i][j] = P[j][i] = MIN_COVARIANCE;
            else
                P[i][j] = P[j][i] = p;
        }
    }
}

void Kalman::ExternalizeState(state_t *state, const vec3f_t *accel) {
    // position state is already in world frame
    state->position = (position_t) {
        S[KC_STATE_X],
        S[KC_STATE_Y],
        S[KC_STATE_Z],
    };

    // velocity is in body frame and needs to be rotated to world frame
    state->velocity = (velocity_t) {
        R[0][0] * S[KC_STATE_PX] + R[0][1] * S[KC_STATE_PY] + R[0][2] * S[KC_STATE_PZ],
        R[1][0] * S[KC_STATE_PX] + R[1][1] * S[KC_STATE_PY] + R[1][2] * S[KC_STATE_PZ],
        R[2][0] * S[KC_STATE_PX] + R[2][1] * S[KC_STATE_PY] + R[2][2] * S[KC_STATE_PZ],
    };

    // Accelerometer measurements are in the body frame and need to be rotated to world frame.
    // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
    // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
    state->accel = (accel_t) {
        R[0][0] * accel->x + R[0][1] * accel->y + R[0][2] * accel->z,
        R[1][0] * accel->x + R[1][1] * accel->y + R[1][2] * accel->z,
        R[2][0] * accel->x + R[2][1] * accel->y + R[2][2] * accel->z - 1.0f,
    };

    // convert the new attitude into Euler YPR
    float yaw = atan2f(
        2 * (q[1] * q[2] + q[0] * q[3]),
        q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]
    );
    float pitch = asinf(-2 * (q[1] * q[3] - q[0] * q[2]));
    float roll = atan2f(
        2 * (q[2] * q[3]+q[0] * q[1]),
        q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
    );

    // Save attitude, adjusted for the legacy CF2 body coordinate system
    state->attitude = (attitude_t) {
        degrees(roll),
        degrees(pitch),
        degrees(yaw)
    };

    // Save quaternion, hopefully one day this could be used in a better controller.
    // Note that this is not adjusted for the legacy coordinate system
    state->attitudeQuat = (quaternion_t) { q[0], q[1], q[2], q[3] };
}

void Kalman::ScalarUpdate(dspm::Mat *Hm, float error, float stdMeasNoise) {
    dspm::Mat HTm = Hm->t();
    dspm::Mat PHTm = Pm * HTm;

    float R = stdMeasNoise * stdMeasNoise;
    float HPHR = R;
    for (int i = 0; i < KC_STATE_DIM; i++) {
        HPHR += PHTm(i, 0) * (*Hm)(0, i);
    }

    dspm::Mat K = PHTm / HPHR;
    for (int i = 0; i < KC_STATE_DIM; i++) {
        S[i] += K(i, 0) * error;
    }

    // Pm = (I - K * H) * Pm * (I - K * H)^T + K * R * K^T
    dspm::Mat I_KH = dspm::Mat::eye(KC_STATE_DIM) - K * (*Hm);;
    Pm = I_KH * Pm * I_KH.t() + (K * R) * K.t();

    _CapCovariance();
}

bool Kalman::CheckBounds() {
    return true;
    // comment
    float maxPosition = 50;
    float maxVelocity = 5;
    for (int i = 0; i < 3; i++) {
        if (fabsf(S[KC_STATE_X + i]) > maxPosition) {
            return false;
        }
        if (fabsf(S[KC_STATE_PX + i]) > maxVelocity) {
            return false;
        }
    }
    return true;
}

// Cholesky Decomposition for a nxn psd matrix (from scratch)
// Reference: https://www.geeksforgeeks.org/cholesky-decomposition-matrix-decomposition/
static void Cholesky_Decomposition(int n, float *matrix, float *lower){
    // Decomposing a matrix into Lower Triangular
    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0.0;
            if (j == i) {
                // summation for diagnols
                for (int k = 0; k < j; k++)
                    sum += powf(lower[j * n + k], 2);
                lower[j * n + j] = sqrtf(matrix[j * n + j] - sum);
            } else {
                for (int k = 0; k < j; k++)
                    sum += (lower[i * n + k] * lower[j * n + k]);
                lower[i * n + j] = (matrix[i * n + j] - sum) / lower[j * n + j];
            }
        }
    }
}

static float GM_UWB(float e) {
    float sigma = 2.0;
    float GM_dn = sigma + e * e;
    return (sigma * sigma) / (GM_dn * GM_dn);
}

static float GM_state(float e) {
    float sigma = 1.5;
    float GM_dn = sigma + e * e;
    return (sigma * sigma) / (GM_dn * GM_dn);
}

#define MAX_ITER (2) // maximum iteration is set to 2.
#define UPPER_BOUND (100)
#define LOWER_BOUND (-100)

// x_err comes from the KF update is the state of error state Kalman filter, set to be zero initially
static float x_err[KC_STATE_DIM] = {0.0};
static dspm::Mat x_errm = dspm::Mat(x_err, KC_STATE_DIM, 1);
static int waitCount = 20;
void Kalman::RobustTdoaUpdate(estimatorPacket_t *packet) {
    if (waitCount > 0) {
        waitCount--;
        return;
    }

	float measurement = 0.0f;
    float x = S[KC_STATE_X];
    float y = S[KC_STATE_Y];
    float z = S[KC_STATE_Z];

    float x0 = packet->tdoa.anchorPositions[0].x, y0 = packet->tdoa.anchorPositions[0].y, z0 = packet->tdoa.anchorPositions[0].z;
    float x1 = packet->tdoa.anchorPositions[1].x, y1 = packet->tdoa.anchorPositions[1].y, z1 = packet->tdoa.anchorPositions[1].z;

    float dx0 = x - x0, dy0 = y - y0, dz0 = z - z0;
    float dx1 = x - x1, dy1 = y - y1, dz1 = z - z1;
    
    float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
    float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));

    // if measurements make sense
    if ((d0 != 0.0f) && (d1 != 0.0f)) {
        float predicted = d1 - d0;
        measurement = packet->tdoa.distanceDiff;

        // innovation term based on prior x
        float error_check = measurement - predicted;    // innovation term based on prior state
        // ---------------------- matrix defination ----------------------------- //
        float P_chol[KC_STATE_DIM][KC_STATE_DIM] = {0.0}; // lower triangular matrix
        dspm::Mat Pc_m = dspm::Mat((float *)P_chol, KC_STATE_DIM, KC_STATE_DIM);
        dspm::Mat Hm = dspm::Mat(1, KC_STATE_DIM);

        // ------------------- Initialization -----------------------//
        float P_iter[KC_STATE_DIM][KC_STATE_DIM];
        memcpy(P_iter, this->P, sizeof(P_iter));                 // init P_iter as P_prior

        float R_iter = packet->tdoa.stdDev * packet->tdoa.stdDev;                    // measurement covariance
        float X_state[KC_STATE_DIM] = {0.0};
        memcpy(X_state, this->S, sizeof(X_state));                     // copy Xpr to X_State and then update in each iterations

        dspm::Mat Kwm, P_w_m;

        // ---------------------- Start iteration ----------------------- //
        for (int iter = 0; iter < MAX_ITER; iter++) {
            // cholesky decomposition for the prior covariance matrix
            Cholesky_Decomposition(KC_STATE_DIM, (float *)P_iter, (float *)P_chol);      // P_chol is a lower triangular matrix
            dspm::Mat Pc_tran_m = Pc_m.t();

            // decomposition for measurement covariance (scalar case)
            float R_chol = sqrtf(R_iter);
            // construct H matrix
            // X_state updates in each iteration
            float x_iter = X_state[KC_STATE_X],  y_iter = X_state[KC_STATE_Y], z_iter = X_state[KC_STATE_Z];

            dx1 = x_iter - x1; dy1 = y_iter - y1; dz1 = z_iter - z1;
            dx0 = x_iter - x0; dy0 = y_iter - y0; dz0 = z_iter - z0;

            d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
            d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

            float predicted_iter = d1 - d0;                           // predicted measurements in each iteration based on X_state
            float error_iter = measurement - predicted_iter;          // innovation term based on iterated X_state
            float e_y = error_iter;
            if (d0 != 0.0f && d1 != 0.0f) {
                // measurement Jacobian changes in each iteration w.r.t linearization point [x_iter, y_iter, z_iter]
                Hm(0, KC_STATE_X) = (dx1 / d1 - dx0 / d0);
                Hm(0, KC_STATE_Y) = (dy1 / d1 - dy0 / d0);
                Hm(0, KC_STATE_Z) = (dz1 / d1 - dz0 / d0);

                if (fabsf(R_chol - 0.0f) < 0.0001f){
                    e_y = error_iter / 0.0001f;
                } else {
                    e_y = error_iter / R_chol;
                }

                // Make sure P_chol, lower trangular matrix, is numerically stable
                for (int col = 0; col < KC_STATE_DIM; col++) {
                    for (int row = col; row < KC_STATE_DIM; row++) {
                        if (isnan(P_chol[row][col]) || P_chol[row][col] > UPPER_BOUND) {
                            P_chol[row][col] = UPPER_BOUND;
                        } else if(row != col && P_chol[row][col] < LOWER_BOUND){
                            P_chol[row][col] = LOWER_BOUND;
                        } else if(row == col && P_chol[row][col] < 0.0f){
                            P_chol[row][col] = 0.0f;
                        }
                    }
                }

                // Matrix inversion is numerically sensitive.
                // Add small values on the diagonal of P_chol to avoid numerical problems.
                float dummy_value = 1e-9f;
                for (int k = 0; k < KC_STATE_DIM; k++){
                    P_chol[k][k] = P_chol[k][k] + dummy_value;
                }
                // keep P_chol
                // inverse is really slow, so we use solve instead
                // dspm::Mat Pc_inv_m = dspm::Mat(Pc_m).inverse();
                // dspm::Mat e_x_m = Pc_inv_m * x_errm;                  // e_x_m = Pc_inv_m.dot(x_errm)
                dspm::Mat e_x_m = dspm::Mat::solve(dspm::Mat(Pc_m), x_errm); // e_x_m = Pc_inv_m.dot(x_errm)

                // compute w_x, w_y --> weighting matrix
                // Since w_x is diagnal matrix, compute the inverse directly
                dspm::Mat wx_invm = dspm::Mat(KC_STATE_DIM, KC_STATE_DIM);
                for (int state_k = 0; state_k < KC_STATE_DIM; state_k++){
                    wx_invm(state_k, state_k) = (float)1.0 / GM_state(e_x_m(state_k, 0));
                }

                // rescale covariance matrix P
                dspm::Mat Pc_w_invm = Pc_m * wx_invm;                 // Pc_w_invm = P_chol.dot(linalg.inv(w_x))
                P_w_m = Pc_w_invm * Pc_tran_m;              // P_w_m = Pc_w_invm.dot(Pc_tran_m) = P_chol.dot(linalg.inv(w_x)).dot(P_chol.T)
                // rescale R matrix
                float w_y = 0.0, R_w = 0.0f;
                w_y = GM_UWB(e_y);                                    // compute the weighted measurement error: w_y
                if (fabsf(w_y - 0.0f) < 0.0001f){
                    R_w = (R_chol * R_chol) / 0.0001f;
                } else {
                    R_w = (R_chol * R_chol) / w_y;
                }

                // ====== INNOVATION COVARIANCE ====== //
                dspm::Mat HTm = Hm.t();                                 // HTm = H.T

                dspm::Mat PHTm = P_w_m * HTm;                          // PHTm = P_w.dot(H.T). The P is the updated P_w
                float HPHR = (Hm * PHTm)(0, 0) + R_w;                          // HPH' + R.            The R is the updated R_w

                // ====== MEASUREMENT UPDATE ======
                // Calculate the Kalman gain and perform the state update
                Kwm = PHTm / HPHR;                             // K_w = PHT / (HPH' + R)
                for (int i = 0; i < KC_STATE_DIM; i++) {
                    //[Note]: The error_check here is the innovation term based on prior state, which doesn't change during iterations.
                    x_err[i] = Kwm(i, 0) * error_check;                   // error state for next iteration
                    X_state[i] = this->S[i] + x_err[i];               // convert to nominal state
                }
                // update P_iter matrix and R matrix for next iteration
                memcpy(P_iter, P_w_m.data, sizeof(P_iter));
                R_iter = R_w;
            }
        }
        // After n iterations, we obtain the rescaled (1) P = P_iter, (2) R = R_iter, (3) Kw.
        // Call the kalman update function with weighted P, weighted K, h, and error_check
        UpdateWithPKE(&Hm, &Kwm, &P_w_m, error_check);
    }
}

void Kalman::UpdateWithPKE(dspm::Mat *Hm, dspm::Mat *Kwm, dspm::Mat *P_w_m, float error) {
    for (int i = 0; i < KC_STATE_DIM; i++) {
        this->S[i] = this->S[i] + (*Kwm)(i, 0) * error; // update state
    }

    // update covariance
    dspm::Mat tmpNN1m = (*Kwm) * ((*Hm) * -1.0f) + dspm::Mat::eye(KC_STATE_DIM);
    this->Pm = tmpNN1m * (*P_w_m); // Ppo = (I - K * H) * P

    _CapCovariance();
}

void Kalman::TdoaUpdate(estimatorPacket_t *packet) {
    float x = S[KC_STATE_X];
    float y = S[KC_STATE_Y];
    float z = S[KC_STATE_Z];

    float x0 = packet->tdoa.anchorPositions[0].x, y0 = packet->tdoa.anchorPositions[0].y, z0 = packet->tdoa.anchorPositions[0].z;
    float x1 = packet->tdoa.anchorPositions[1].x, y1 = packet->tdoa.anchorPositions[1].y, z1 = packet->tdoa.anchorPositions[1].z;

    float dx0 = x - x0, dy0 = y - y0, dz0 = z - z0;
    float dx1 = x - x1, dy1 = y - y1, dz1 = z - z1;
    
    float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
    float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
    float predicted = d0 - d1;
    float measurement = packet->tdoa.distanceDiff;
    float error = measurement - predicted;

    dspm::Mat Hm = dspm::Mat(1, KC_STATE_DIM);

    if (d0 != 0.0f && d1 != 0.0f) {
        // measurement Jacobian
        Hm(0, KC_STATE_X) = dx0 / d0 - dx1 / d1;
        Hm(0, KC_STATE_Y) = dy0 / d0 - dy1 / d1;
        Hm(0, KC_STATE_Z) = dz0 / d0 - dz1 / d1;

        // update the Kalman filter with the new measurement
        ScalarUpdate(&Hm, error, packet->tdoa.stdDev);
    }
}
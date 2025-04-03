#include "kalman_core.h"
#include <string.h>
#include <math.h>
#include "utils.h"

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100.0f)
#define MIN_COVARIANCE (1e-6f)
#define EPSILON        (1e-6f)

// Initial variances, uncertain of position, but know we're stationary and roughly flat
static const float stdDevInitPos_xy = 100;
static const float stdDevInitPos_z = 1;
static const float stdDevInitVel_xyz = 0.01;
static const float stdDevInitAtti_rpy = 0.01;

static float procNoiseAcc_xy = 0.2f;
static float procNoiseAcc_z = 0.3f;
static float procNoiseVel = 0.02f;
static float procNoisePos = 0.01f;
static float procNoiseAtt = 0.005f;
// static float measNoiseGyro_roll_pitch = 0.1f; // radians per second
// static float measNoiseGyro_yaw = 0.1f; // radians per second
static float measNoiseGyro_roll_pitch = 0.000041f;  // in rad/s
static float measNoiseGyro_yaw       = 0.000041f;  // in rad/s
static float measNoiseAccel_xy       = 0.00047f;   // in m/s²
static float measNoiseAccel_z        = 0.00047f;   // in m/s²

Kalman::Kalman() {
    // Initialize the Kalman filter state
    memset(S, 0, sizeof(S));
    memset(q, 0, sizeof(q));
    memset(R, 0, sizeof(R));
    memset(P, 0, sizeof(P));

    q[0] = 1.0f; // Initialize the quaternion to identity
    R[0][0] = R[1][1] = R[2][2] = 1.0f; // Initialize the rotation matrix to identity

    P[KC_STATE_X][KC_STATE_X] = powf(stdDevInitPos_xy, 2);
    P[KC_STATE_Y][KC_STATE_Y] = powf(stdDevInitPos_xy, 2);
    P[KC_STATE_Z][KC_STATE_Z] = powf(stdDevInitPos_z, 2);
    P[KC_STATE_PX][KC_STATE_PX] = powf(stdDevInitVel_xyz, 2);
    P[KC_STATE_PY][KC_STATE_PY] = powf(stdDevInitVel_xyz, 2);
    P[KC_STATE_PZ][KC_STATE_PZ] = powf(stdDevInitVel_xyz, 2);
    P[KC_STATE_D0][KC_STATE_D0] = powf(stdDevInitAtti_rpy, 2);
    P[KC_STATE_D1][KC_STATE_D1] = powf(stdDevInitAtti_rpy, 2);
    P[KC_STATE_D2][KC_STATE_D2] = powf(stdDevInitAtti_rpy, 2);
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
    // this is the gyroscope angular velocity integrated over the sample period
    float dtwx = dt * gyro->x;
    float dtwy = dt * gyro->y;
    float dtwz = dt * gyro->z;

    // compute the quaternion values in [w,x,y,z] order
    float angle = sqrtf(dtwx * dtwx + dtwy * dtwy + dtwz * dtwz) + EPSILON;
    float ca = cosf(angle / 2.0f);
    float sa = sinf(angle / 2.0f);
    float dq[4] = {ca , sa * dtwx / angle , sa * dtwy / angle , sa * dtwz / angle};

    float tmpq0, tmpq1, tmpq2, tmpq3;
    // rotate the quad's attitude by the delta quaternion vector computed above
    tmpq0 = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
    tmpq1 = dq[1] * q[0] + dq[0] * q[1] + dq[3] * q[2] - dq[2] * q[3];
    tmpq2 = dq[2] * q[0] - dq[3] * q[1] + dq[0] * q[2] + dq[1] * q[3];
    tmpq3 = dq[3] * q[0] + dq[2] * q[1] - dq[1] * q[2] + dq[0] * q[3];

    /* This reversion would cause yaw estimation diminish to zero */
    // if (!isFlying) {
    //     float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;
    //     tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * 1.0;
    //     tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * 0;
    //     tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * 0;
    //     tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * 0;
    // }

    // normalize and store the result
    float norm = sqrtf(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + EPSILON;

    q[0] = tmpq0 / norm;
    q[1] = tmpq1 / norm;
    q[2] = tmpq2 / norm;
    q[3] = tmpq3 / norm;
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
        P[KC_STATE_D0][KC_STATE_D0] += powf(measNoiseGyro_roll_pitch * dt, 2) 
                                               + powf(procNoiseAtt, 2);
        P[KC_STATE_D1][KC_STATE_D1] += powf(measNoiseGyro_roll_pitch * dt, 2) 
                                               + powf(procNoiseAtt, 2);
        P[KC_STATE_D2][KC_STATE_D2] += powf(measNoiseGyro_yaw * dt, 2) 
                                               + powf(procNoiseAtt, 2);
    }

    // Cap covariance values to ensure numerical stability
    capCovariance();
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
        float angle = sqrtf(v0 * v0 + v1 * v1 + v2 * v2) + EPSILON;
        float ca = cosf(angle / 2.0f);
        float sa = sinf(angle / 2.0f);
        float dq[4] = { ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle };

        // rotate the quad's attitude by the delta quaternion vector computed above
        float tmpq0 = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
        float tmpq1 = dq[1] * q[0] + dq[0] * q[1] + dq[3] * q[2] - dq[2] * q[3];
        float tmpq2 = dq[2] * q[0] - dq[3] * q[1] + dq[0] * q[2] + dq[1] * q[3];
        float tmpq3 = dq[3] * q[0] + dq[2] * q[1] - dq[1] * q[2] + dq[0] * q[3];

        // normalize and store the result
        float norm = sqrtf(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + EPSILON;
        q[0] = tmpq0 / norm;
        q[1] = tmpq1 / norm;
        q[2] = tmpq2 / norm;
        q[3] = tmpq3 / norm;

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

    // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
    R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];

    R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
    R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];

    R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
    R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
    R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    // reset the attitude error
    S[KC_STATE_D0] = 0;
    S[KC_STATE_D1] = 0;
    S[KC_STATE_D2] = 0;

    // enforce symmetry of the covariance matrix, and ensure the values stay bounded
    capCovariance();
}

void Kalman::capCovariance() {
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

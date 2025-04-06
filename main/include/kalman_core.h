/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D'Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D'Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 */

#ifndef __KALMAN_FILTER_CORE_H__
#define __KALMAN_FILTER_CORE_H__
#include "types.h"
#include "esp_dsp.h"

// Indexes to access the quad's state, stored as a column vector
typedef enum {
    KC_STATE_X,  KC_STATE_Y,  KC_STATE_Z,
	KC_STATE_PX, KC_STATE_PY, KC_STATE_PZ,
	KC_STATE_D0, KC_STATE_D1, KC_STATE_D2,
    KC_STATE_DIM
} kalmanCoreStateIdx_t;

#define GRAVITY_EARTH   (9.80665f)

class Kalman {
private:
    /**
     * Quadrocopter State
     *
     * The internally-estimated state is:
     * - X, Y, Z: the quad's position in the global frame
     * - PX, PY, PZ: the quad's velocity in its body frame
     * - D0, D1, D2: attitude error
     *
     * For more information, refer to the paper
     */
    float S[KC_STATE_DIM];

    float q[4];           // The quad's attitude as a quaternion (w,x,y,z)
    float R[3][3];        // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
    float P[KC_STATE_DIM][KC_STATE_DIM]; // The covariance matrix, estimate uncertainty
    dspm::Mat Pm;         // The covariance matrix, estimate uncertainty

    void _CapCovariance();
    void _UpdateRotationMatrix();
    void _DeltaGyro2Quaternion(float d0, float d1, float d2);
public:
    Kalman(vec3f_t initialAccel = {0, 0, 1});
    ~Kalman();

    float *GetState() { return S; }

    void Predict(imu_t* imuData, float dt, bool isFlying);
    void AddProcessNoise(float dt);
    void Finalize();
    void ExternalizeState(state_t *state, const vec3f_t *accel);
    void ScalarUpdate(dspm::Mat *Hm, float error, float stdMeasNoise);
    void UpdateWithPKE(dspm::Mat *Hm, dspm::Mat *Kwm, dspm::Mat *P_w_m, float error);

    bool CheckBounds();

    void RobustTdoaUpdate(estimatorPacket_t *packet);
    void TdoaUpdate(estimatorPacket_t *packet);
};

/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
// void kalmanCorePredict(kalmanCoreData_t* coreData, imu_t* imuData, float dt, bool isFlying);

// void kalmanCoreAddProcessNoise(kalmanCoreData_t* coreData, float dt);

// /*  - Finalization to incorporate attitude error into body attitude */
// void kalmanCoreFinalize(kalmanCoreData_t* coreData);

// /*  - Externalization to move the filter's internal state into the external state expected by other modules */
// void kalmanCoreExternalizeState(const kalmanCoreData_t* coreData, state_t *state, const vec3f_t *acc, uint32_t tick);

// void kalmanCoreScalarUpdate(kalmanCoreData_t* coreData, dspm::Mat *Hm, float error, float stdMeasNoise);

// bool kalmanCoreCheckBounds(kalmanCoreData_t* coreData);

#endif //__KALMAN_FILTER_CORE_H__

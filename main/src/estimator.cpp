#include "estimator.h"
#include "kalman_core.h"
#include "freeRTOS_helper.h"
#include "esp_timer.h"
#include "utils.h"
#include "imu.h"

#define ESTIMATOR_TASK_RATE RATE_1000_HZ
#define ESTIMATOR_PREDICTION_RATE RATE_100_HZ

STATIC_MUTEX_DEF(estimatorDataMutex);
STATIC_QUEUE_DEF(estimatorDataQueue, 30, estimatorPacket_t);

static Kalman kalmanCore;
static state_t stateData;
static imu_t latestImu;

static imu_t imuAccumulator;
static uint32_t imuCount = 0;
static int64_t lastImuPrediction;

static int count = 0;

bool processDataQueue() {
    estimatorPacket_t packet;
    bool update = false;
    
    while (STATIC_QUEUE_RECEIVE(estimatorDataQueue, &packet, 0) == pdTRUE) {
        switch (packet.type) {
            case ESTIMATOR_TYPE_IMU:
                latestImu = packet.imu;
                for (int i = 0; i < 3; i++) {
                    imuAccumulator.gyro.v[i] += packet.imu.gyro.v[i];
                    imuAccumulator.accel.v[i] += packet.imu.accel.v[i];
                }
                imuCount++;
                break;
            case ESTIMATOR_TYPE_UWB:
                // kalmanCore.TdoaUpdate(&packet);
                // update = true;
                break;
            default:
                break;
        }
    }

    int64_t currentTime = esp_timer_get_time();
    int64_t dt = currentTime - lastImuPrediction;
    if (imuCount > 0 && dt >= 1000000 / ESTIMATOR_PREDICTION_RATE) {
        for (int i = 0; i < 3; i++) {
            imuAccumulator.gyro.v[i] = radians(imuAccumulator.gyro.v[i] / imuCount);
            imuAccumulator.accel.v[i] = imuAccumulator.accel.v[i] * GRAVITY_EARTH / imuCount;
        }

        kalmanCore.Predict(&imuAccumulator, dt / 1000000.0f, false);

        imuCount = 0;
        imuAccumulator = imu_t();
        lastImuPrediction = currentTime;
        update = true;
    }

    return update;
}

void estimatorKalmanGetState(state_t *state) {
    STATIC_MUTEX_LOCK(estimatorDataMutex, portMAX_DELAY);
    *state = stateData;
    STATIC_MUTEX_UNLOCK(estimatorDataMutex);
}

void estimatorKalmanEnqueue(estimatorPacket_t *packet) {
    STATIC_QUEUE_SEND(estimatorDataQueue, packet, 0);
}

void estimatorKalmanTask(void *argument) {
    kalmanCore = Kalman(getInitialAccel());

    TASK_TIMER_DEF(ESTIMATOR, ESTIMATOR_TASK_RATE);
    int64_t lastAddNoiseTime = lastImuPrediction = esp_timer_get_time();
    bool update;
    while (1) {
        TASK_TIMER_WAIT(ESTIMATOR);
        update = processDataQueue();

        int64_t currentTime = esp_timer_get_time();
        kalmanCore.AddProcessNoise((currentTime - lastAddNoiseTime) / 1e6f);
        lastAddNoiseTime = currentTime;
        
        if (update) {
            kalmanCore.Finalize();
            if (!kalmanCore.CheckBounds()) {
                float *S = kalmanCore.GetState();
                printf("Kalman Core [RESET]: P(%.3f %.3f %.3f), V(%.3f %.3f %.3f)\n",
                    S[0], S[1], S[2],
                    S[3], S[4], S[5]);
                kalmanCore = Kalman(getInitialAccel());
            }
        }

        STATIC_MUTEX_LOCK(estimatorDataMutex, portMAX_DELAY);
        kalmanCore.ExternalizeState(&stateData, &latestImu.accel);
        STATIC_MUTEX_UNLOCK(estimatorDataMutex);

        if (count % 250 == 0) {
            float *S = kalmanCore.GetState();
            printf("Kalman Core: P(%.3f %.3f %.3f), V(%.3f %.3f %.3f), A(%.3f %.3f %.3f)\n",
                S[0], S[1], S[2],
                S[3], S[4], S[5],
                stateData.attitude.roll, stateData.attitude.pitch, stateData.attitude.yaw);
        }
        count++;
    }
}

void estimatorInit() {
    STATIC_MUTEX_INIT(estimatorDataMutex);
    STATIC_QUEUE_INIT(estimatorDataQueue);
    xTaskCreatePinnedToCore(estimatorKalmanTask, "kalman_task", 8192, NULL, 3, NULL, 0);
}
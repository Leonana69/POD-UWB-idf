#include "estimator.h"
#include "kalman_core.h"
#include "freeRTOS_helper.h"
#include "esp_timer.h"
#include "utils.h"

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
    int64_t currentTime;
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

    currentTime = esp_timer_get_time();
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

void estimatorKalmanUpdate(state_t *state) {
    STATIC_MUTEX_LOCK(estimatorDataMutex, portMAX_DELAY);
    *state = stateData;
    STATIC_MUTEX_UNLOCK(estimatorDataMutex);
}

void estimatorKalmanEnqueue(estimatorPacket_t *packet) {
    STATIC_QUEUE_SEND(estimatorDataQueue, packet, 0);
}

void estimatorKalmanTask(void *argument) {\
    TASK_TIMER_DEF(ESTIMATOR, ESTIMATOR_TASK_RATE);
    int64_t lastTime = esp_timer_get_time();
    bool update;

    while (1) {
        TASK_TIMER_WAIT(ESTIMATOR);

        update = processDataQueue();

        int64_t currentTime = esp_timer_get_time();
        kalmanCore.AddProcessNoise((currentTime - lastTime) / 1e6f);
        lastTime = currentTime;
        
        if (update) {
            kalmanCore.Finalize();
            if (!kalmanCore.CheckBounds()) {
                float *S = kalmanCore.GetState();
                printf("Kalman Core [RESET]: P(%.3f %.3f %.3f), V(%.3f %.3f %.3f)\n",
                    S[1], S[0], S[2],
                    S[3], S[4], S[5]);
                kalmanCore = Kalman();
            }
        }

        STATIC_MUTEX_LOCK(estimatorDataMutex, portMAX_DELAY);
        kalmanCore.ExternalizeState(&stateData, &latestImu.accel);
        STATIC_MUTEX_UNLOCK(estimatorDataMutex);

        if (count++ % 250 == 0) {
            printf("State: %.2f %.2f %.2f, %.2f %.2f %.2f, %.2f %.2f %.2f\n",
                stateData.position.x, stateData.position.y, stateData.position.z,
                stateData.velocity.x, stateData.velocity.y, stateData.velocity.z,
                stateData.attitude.roll, stateData.attitude.pitch, stateData.attitude.yaw);
        }
    }
}

void estimatorInit() {
    STATIC_MUTEX_INIT(estimatorDataMutex);
    STATIC_QUEUE_INIT(estimatorDataQueue);
    lastImuPrediction = esp_timer_get_time();

    xTaskCreatePinnedToCore(estimatorKalmanTask, "kalman_task", 8192, NULL, 3, NULL, 0);
}
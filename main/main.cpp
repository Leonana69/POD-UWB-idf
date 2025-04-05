/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "button.h"
#include "wifi_link.h"
#include "imu.h"
#include "loco.h"
#include "estimator.h"
extern "C" void app_main();

#include "kalman_core.h"
#include "esp_dsp.h"

void app_main() {
    // estimatorPacket_t packet;
    // packet.type = ESTIMATOR_TYPE_UWB;
    // packet.tdoa.distanceDiff = 0.5f;
    // packet.tdoa.stdDev = 0.15f;
    // packet.tdoa.anchorIds[0] = 0;
    // packet.tdoa.anchorIds[1] = 1;
    // packet.tdoa.anchorPositions[0].x = -1.0f;
    // packet.tdoa.anchorPositions[0].y = 0.0f;
    // packet.tdoa.anchorPositions[0].z = 0.0f;
    // packet.tdoa.anchorPositions[1].x = 2.0f;
    // packet.tdoa.anchorPositions[1].y = 0.0f;
    // packet.tdoa.anchorPositions[1].z = 0.0f;

    // Kalman kalmanCore;
    
    // imu_t imuData;
    // imuData.gyro.x = 0.0f;
    // imuData.gyro.y = 0.0f;
    // imuData.gyro.z = 0.0f;
    // imuData.accel.x = 0.0f;
    // imuData.accel.y = 0.0f;
    // imuData.accel.z = 1.0f;

    // float *state = kalmanCore.GetState();
    // for (int i = 0; i < 20; i++) {
    //     kalmanCore.AddProcessNoise(0.001f);
    //     if (i % 10 == 0) {
    //         packet.tdoa.distanceDiff = i % 3 ? 0.5f + (float) i / 100.0f : 0.5f - (float) i / 100.0f;
    //         kalmanCore.RobustTdoaUpdate(&packet);
    //         kalmanCore.Predict(&imuData, 0.01f, false);
    //         kalmanCore.Finalize();
    //     }
    // }

    // for (int i = 0; i < KC_STATE_DIM; i++) {
    //     printf("State[%d]: %f\n", i, state[i]);
    // }

    // return;

    printf("System Starting...\n");
    buttonInit();
    // wifiInit(-1);
    // wifiLinkInit();
    imuInit();
    estimatorInit();
    dw1000_init();
}
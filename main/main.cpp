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
    // Kalman kalmanCore = Kalman(vec3f_t{0, 0, 1});
    // dspm::Mat Hm = dspm::Mat(1, KC_STATE_DIM);
    // Hm(0, KC_STATE_X) = 10.0f;
    // imu_t imuData;
    // for (int i = 0; i < 5; i++) {
    //     imuData.accel.x = i * 0.1f;
    //     imuData.accel.y = i * 0.2f;
    //     imuData.accel.z = i * 0.3f;
    //     imuData.gyro.x = i * 0.05f;
    //     imuData.gyro.y = i * 0.15f;
    //     imuData.gyro.z = i * 0.25f;
    //     kalmanCore.AddProcessNoise(0.1f);
    //     kalmanCore.ScalarUpdate(&Hm, 0.1f, 0.2f);
    //     kalmanCore.Predict(&imuData, 0.1f, false);
    //     kalmanCore.Finalize();
    // }

    // printf("%.3f %.3f %.3f\n", kalmanCore.S[0], kalmanCore.S[1], kalmanCore.S[2]);

    // printf("%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n",
    //     kalmanCore.R[0][0], kalmanCore.R[0][1], kalmanCore.R[0][2],
    //     kalmanCore.R[1][0], kalmanCore.R[1][1], kalmanCore.R[1][2],
    //     kalmanCore.R[2][0], kalmanCore.R[2][1], kalmanCore.R[2][2]
    // );

    // for (int i = 0; i < 9; i++) {
    //     for (int j = 0; j < 9; j++) {
    //         printf("%.3f ", kalmanCore.Pm(i, j));
    //     }
    //     printf("\n");
    // }

    // while (true) {
    //     vTaskDelay(1000);
    // }
    printf("System Starting...\n");
    buttonInit();
    // wifiInit(-1);
    // wifiLinkInit();
    imuInit();
    estimatorInit();
    dw1000_init();
}
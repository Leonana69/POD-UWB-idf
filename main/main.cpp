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

void app_main() {
    printf("System Starting...\n");
    buttonInit();
    wifiInit(-1);
    wifiLinkInit();
    imuInit();
    estimatorInit();
    dw1000_init();
}
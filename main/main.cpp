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

void app_main(void) {
    printf("System Starting...\n");
    buttonInit();
    wifiInit(-1);
    wifiLinkInit();
    imuInit();
    dw1000_init();
}
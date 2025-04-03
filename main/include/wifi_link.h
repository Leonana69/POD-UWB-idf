#ifndef __WIFI_LINK_H__
#define __WIFI_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sys/socket.h"

void wifiInit(int8_t configIndex);
void wifiLinkInit();

#ifdef __cplusplus
}
#endif

#endif // __WIFI_LINK_H__
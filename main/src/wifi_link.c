#include "wifi_link.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "sys/socket.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "freertos/event_groups.h"

static bool wifiConnected = false;

wifi_config_t wifiConfigs[3] = {
    {
        .sta = {
            .ssid = "YECL-DEMO",
            .password = "64221771",
        },
    },
    {
        .sta = {
            .ssid = "YECL-tplink",
            .password = "08781550",
        },
    },
    {
        .sta = {
            .ssid = "LEONA",
            .password = "64221771",
        },
    }
};

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            printf("Retry to connect to the AP\n");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        vTaskDelay(10);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("Get IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // Get the signal strength (RSSI)
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            printf("Signal Strength (RSSI): %d dBm\n", ap_info.rssi);
        } else {
            printf("Failed to get signal strength\n");
        }
    }
}

int8_t wifiScan() {
    int8_t configIndex = -1;
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false
    };

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    uint16_t ap_count = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));

    for (int j = 0; j < 3; j++) {
        bool found = false;
        for (int i = 0; i < ap_count; i++) {
            if (strcmp((char *) ap_list[i].ssid, (char *) wifiConfigs[j].sta.ssid) == 0) {
                configIndex = j;
                found = true;
                break;
            }
        }
        if (found)
            break;
    }
    
    free(ap_list);
    return configIndex;
}

void wifiInit(int8_t configIndex) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Disable power saving mode
    
    ESP_ERROR_CHECK(esp_wifi_start());
    // ESP_ERROR_CHECK(esp_wifi_set_channel(8, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT20));
    
    if (configIndex < 0 || configIndex > 2) {
        configIndex = wifiScan();
    }

    if (configIndex < 0) {
        printf("Cannot find any known AP\n");
        return;
    }

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifiConfigs[configIndex]));
    ESP_ERROR_CHECK(esp_wifi_start());
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        printf("Connecting to %s [OK]\n", (char *) wifiConfigs[configIndex].sta.ssid);
        wifiConnected = true;
    } else if (bits & WIFI_FAIL_BIT) {
        printf("Connecting to %s [FAILED]\n", (char *) wifiConfigs[configIndex].sta.ssid);
    } else {
        printf("UNEXPECTED EVENT\n");
    }
}

void wifiRSSITask(void *pvParameters) {
    while (true) {
        if (wifiConnected) {
            wifi_ap_record_t ap_info;
            esp_wifi_sta_get_ap_info(&ap_info);
            printf("RSSI: %d dBm\n", ap_info.rssi);
        }
        vTaskDelay(1000);
    }
}

void wifiLinkInit() {
    if (!wifiConnected)
        return;

    // xTaskCreatePinnedToCore(wifiRSSITask, "wifi_rssi_task", 4096, NULL, 10, NULL, 1);
}
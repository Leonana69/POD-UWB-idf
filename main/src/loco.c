#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "loco.h"
#include "driver/gpio.h"
#include "libdw1000.h"
#include "tdoa2.h"

static spi_device_handle_t spi_handle;
static bool isInit = false;
dwDevice_t dwm_device;
dwDevice_t *dwm = &dwm_device;
static uwbAlgorithm_t *uwbAlgorithm = &uwbTdoa2TagAlgorithm;

TaskHandle_t uwbTaskHandle;
static void uwbInterruptHandler(void *arg);
static void uwbTask(void *pvParameters);

/// @brief SPI and GPIO setup ///
static void init_spi() {
    spi_bus_config_t buscfg = {};
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("Initialize SPI2 bus [FAILED]\n");
        return;
    } else {
        printf("Initialize SPI2 bus [OK]\n");
    }

    // Configure SPI device interface
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                        // SPI mode 0
        .clock_speed_hz = 2 * 1000 * 1000, // 1MHz clock speed
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .input_delay_ns = 0
    };

    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        printf("Add SPI device [FAILED]\n");
        return;
    }

    printf("SPI Init [OK]\n");
}

void init_irq_rst() {
    esp_err_t ret;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        printf("Configure GPIO IRQ [FAILED]");
        return;
    }

    ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    ret |= gpio_isr_handler_add(PIN_NUM_IRQ, uwbInterruptHandler, NULL);
    if (ret != ESP_OK) {
        printf("Install ISR [FAILED]");
        return;
    }

    // Configure GPIO for reset pin
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_RST),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&rst_conf);
    if (ret != ESP_OK) {
        printf("Configure GPIO RST [FAILED]");
        return;
    }
}

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
    const void* data, size_t dataLength) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    memcpy(spiTxBuffer, header, headerLength);
    memcpy(spiTxBuffer + headerLength, data, dataLength);
    t.tx_buffer = spiTxBuffer;
    t.rx_buffer = spiRxBuffer;
    t.length = (headerLength + dataLength) * 8;
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        printf("SPI write [FAILED]\n");
    }
}

static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
    void* data, size_t dataLength) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    memcpy(spiTxBuffer, header, headerLength);
    memset(spiTxBuffer + headerLength, 0, dataLength);
    t.tx_buffer = spiTxBuffer;
    t.rx_buffer = spiRxBuffer;
    t.length = (headerLength + dataLength) * 8; // length in bits
    t.rxlength = t.length; // rx length in bits

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        printf("SPI read [FAILED]\n");
        return;
    }

    memcpy(data, spiRxBuffer + headerLength, dataLength);
}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed) {
    uint32_t spiSpeed = 0;
    if (speed == dwSpiSpeedLow) {
        spiSpeed = 2 * 1000 * 1000;
    } else if (speed == dwSpiSpeedHigh) {
        spiSpeed = 20 * 1000 * 1000;
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = spiSpeed,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .input_delay_ns = 0,
    };

    esp_err_t ret = spi_bus_remove_device(spi_handle);
    if (ret != ESP_OK) {
        printf("Set speed: remove SPI device [FAILED]\n");
        return;
    }

    ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        printf("Add SPI device with new speed [FAILED]\n");
    }
}

static void delayms(dwDevice_t* dev, unsigned int delay) {
    vTaskDelay(delay);
}

static dwOps_t dwOps = {
    .spiRead = spiRead,
    .spiWrite = spiWrite,
    .spiSetSpeed = spiSetSpeed,
    .delayms = delayms,
};

static void txCallback(dwDevice_t *dev) {
    uwbAlgorithm->onEvent(dev, eventPacketSent);
}

static void rxCallback(dwDevice_t *dev) {
    uwbAlgorithm->onEvent(dev, eventPacketReceived);
}

static void rxTimeoutCallback(dwDevice_t *dev) {
    uwbAlgorithm->onEvent(dev, eventReceiveTimeout);
}

static void rxFailedCallback(dwDevice_t *dev) {
    uwbAlgorithm->onEvent(dev, eventReceiveFailed);
}

/// @brief dw1000 setup ///
void dw1000_init() {
    init_spi();
    init_irq_rst();
    
    // Reset the DW1000
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(10);

    // TODO: check the pin status with oscilloscope
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(20);

    dwInit(dwm, &dwOps);

    int ret = dwConfigure(dwm);
    if (ret != 0) {
        isInit = false;
        printf("Failed to configure DW1000\n");
        return;
    }

    dwEnableAllLeds(dwm);

    dwTime_t delay = {.full = 0};
    dwSetAntenaDelay(dwm, delay);

    dwAttachSentHandler(dwm, txCallback);
    dwAttachReceivedHandler(dwm, rxCallback);
    dwAttachReceiveFailedHandler(dwm, rxFailedCallback);
    dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);

    dwNewConfiguration(dwm);
    dwSetDefaults(dwm);

    // set the mode for different ranging
    // dwEnableMode(dwm, MODE_SHORTDATA_MID_ACCURACY);
    dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);

    dwSetChannel(dwm, CHANNEL_2);
    dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

    dwUseSmartPower(dwm, true);

    dwSetReceiveWaitTimeout(dwm, 10000);
    dwCommitConfiguration(dwm);

    xTaskCreatePinnedToCore(uwbTask, "loco_service_task", 8192, NULL, 20, &uwbTaskHandle, 1);
    printf("DW1000 Init [OK]\n");
    isInit = true;
}

void uwbInterruptHandler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(uwbTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void uwbTask(void *pvParameters) {
    uwbAlgorithm->init(dwm);
    uwbAlgorithm->onEvent(dwm, eventTimeout);

    while (true) {
        BaseType_t ret = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        if (ret == pdTRUE) {
            do {
                dwHandleInterrupt(dwm);
            } while (gpio_get_level(PIN_NUM_IRQ) == 1);
        } else {
            uwbAlgorithm->onEvent(dwm, eventTimeout);
        }
    }
}
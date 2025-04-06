#include "imu.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <string.h>

#include "types.h"
#include "filter.h"
#include "bmi270.h"
#include "freeRTOS_helper.h"
#include "estimator.h"

#define IMU_TASK_RATE 1000 // Hz
STATIC_MUTEX_DEF(imuDataMutex);

static struct bmi2_dev bmi2Dev;
enum { ACCEL = 0, GYRO = 1 };
struct bmi2_sens_config bmi270Config[2];
static float accelValue2Gravity;
static float gyroValue2Degree;
static imu_t imuData;

static struct bmi2_sensor_data bmi270Data[2] = {
    { .type = BMI2_ACCEL },
    { .type = BMI2_GYRO }
};

static lpf2pData lpf2pAccel[3];
static lpf2pData lpf2pGyro[3];
static void applyLpf(lpf2pData *lpfData, vec3f_t *data) {
    for (uint8_t i = 0; i < 3; i++)
        data->v[i] = lpf2pApply(&lpfData[i], data->v[i]);
}

static spi_device_handle_t imu_spi_handle;

/// @brief SPI and GPIO setup ///
static void init_spi() {
    spi_bus_config_t buscfg = {};
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
    buscfg.miso_io_num = IMU_SPI_MISO_PIN;
    buscfg.mosi_io_num = IMU_SPI_MOSI_PIN;
    buscfg.sclk_io_num = IMU_SPI_SCK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;
    esp_err_t ret = spi_bus_initialize(IMU_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("Initialize SPI3 bus [FAILED]\n");
        return;
    }

    // Configure SPI device interface
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                        // SPI mode 0
        .clock_speed_hz = 4 * 1000 * 1000, // 1MHz clock speed
        .spics_io_num = IMU_SPI_CS_PIN,
        .queue_size = 1,
        .input_delay_ns = 0
    };

    ret = spi_bus_add_device(IMU_SPI_HOST, &devcfg, &imu_spi_handle);
    if (ret != ESP_OK) {
        printf("Add SPI device [FAILED]\n");
    }
}

static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static int8_t spiWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    spiTxBuffer[0] = reg_addr;
    memcpy(spiTxBuffer + 1, data, len);
    t.tx_buffer = spiTxBuffer;
    t.rx_buffer = spiRxBuffer;
    t.length = (1 + len) * 8;
    esp_err_t ret = spi_device_transmit(imu_spi_handle, &t);
    if (ret != ESP_OK) {
        printf("SPI write [FAILED]\n");
    }
    return ret;
}

static int8_t spiRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    spiTxBuffer[0] = reg_addr;
    memset(spiTxBuffer + 1, 0x00, len);  // Send dummy bytes
    t.tx_buffer = spiTxBuffer;
    t.rx_buffer = spiRxBuffer;
    t.length = (1 + len) * 8;
    t.rxlength = t.length;

    esp_err_t ret = spi_device_transmit(imu_spi_handle, &t);
    if (ret != ESP_OK) {
        printf("SPI read [FAILED]\n");
        return ret;
    }

    memcpy(data, spiRxBuffer + 1, len);
    return ret;
}

static void sensorsDelayUs(uint32_t period, void *intf_ptr) {
    if (period < 20) {
        esp_rom_delay_us(period);
        return;
    }
    vTaskDelay((period + 999) / 1000);
}

static vec3f_t gyroBias;
static vec3f_t initial_accel;
static float accelScale;

static bool isInit = false;
vec3f_t getInitialAccel() {
    while (!isInit) {
        vTaskDelay(5);
    }
    return initial_accel;
}

static bool imuCalibration() {
    const int caliNbr = 512;
    gyroBias = (vec3f_t){ 0 };
    initial_accel = (vec3f_t){ 0 };
    vec3f_t gyroBiasSquared = (vec3f_t){ 0 };
    accelScale = 0;
    
    for (int i = 0; i < caliNbr; i++) {
        bmi2_get_sensor_data(&bmi270Data[ACCEL], 1, &bmi2Dev);
		bmi2_get_sensor_data(&bmi270Data[GYRO], 1, &bmi2Dev);

        float ax = bmi270Data[ACCEL].sens_data.acc.x;
        float ay = bmi270Data[ACCEL].sens_data.acc.y;
        float az = bmi270Data[ACCEL].sens_data.acc.z;
        initial_accel.x += ax;
        initial_accel.y += ay;
        initial_accel.z += az;

        float gx = bmi270Data[GYRO].sens_data.gyr.x;
        float gy = bmi270Data[GYRO].sens_data.gyr.y;
        float gz = bmi270Data[GYRO].sens_data.gyr.z;

        gyroBias.x += gx;
        gyroBias.y += gy;
        gyroBias.z += gz;
        gyroBiasSquared.x += gx * gx;
        gyroBiasSquared.y += gy * gy;
        gyroBiasSquared.z += gz * gz;
        vTaskDelay(1);
    }

    initial_accel.x /= caliNbr;
    initial_accel.y /= caliNbr;
    initial_accel.z /= caliNbr;
    accelScale = sqrtf(initial_accel.x * initial_accel.x + 
                        initial_accel.y * initial_accel.y + 
                        initial_accel.z * initial_accel.z);
    initial_accel.x /= accelValue2Gravity;
    initial_accel.y /= accelValue2Gravity;
    initial_accel.z /= accelValue2Gravity;

    gyroBias.x /= caliNbr;
    gyroBias.y /= caliNbr;
    gyroBias.z /= caliNbr;
    gyroBiasSquared.x = gyroBiasSquared.x / caliNbr - gyroBias.x * gyroBias.x;
    gyroBiasSquared.y = gyroBiasSquared.y / caliNbr - gyroBias.y * gyroBias.y;
    gyroBiasSquared.z = gyroBiasSquared.z / caliNbr - gyroBias.z * gyroBias.z;
    const float gyroBiasLimit = 1000;
    if (gyroBiasSquared.x < gyroBiasLimit
     && gyroBiasSquared.y < gyroBiasLimit
     && gyroBiasSquared.z < gyroBiasLimit) {
        printf("IMU Calibration [OK]");
        isInit = true;
        return true;
    } else {
        printf("IMU Calibration [FAILED]");
        return false;
    }
}

void imuTask(void *argument) {
    estimatorPacket_t packet;
    packet.type = ESTIMATOR_TYPE_IMU;
    imu_t imuBuffer = { 0 };
    // systemWaitStart();
    while (!imuCalibration());
    TASK_TIMER_DEF(IMU, IMU_TASK_RATE);
    
    int count = 0;
    while (1) {
        TASK_TIMER_WAIT(IMU);
        bmi2_get_sensor_data(&bmi270Data[ACCEL], 1, &bmi2Dev);
		bmi2_get_sensor_data(&bmi270Data[GYRO], 1, &bmi2Dev);
        
        // imuBuffer.accel.x = (bmi270Data[ACCEL].sens_data.acc.x - accelBias.x) * accelValue2Gravity;
        // imuBuffer.accel.y = (bmi270Data[ACCEL].sens_data.acc.y - accelBias.y) * accelValue2Gravity;
        // imuBuffer.accel.z = (bmi270Data[ACCEL].sens_data.acc.z - accelBias.z) * accelValue2Gravity;

        imuBuffer.accel.x = (bmi270Data[ACCEL].sens_data.acc.x) / accelScale;
        imuBuffer.accel.y = (bmi270Data[ACCEL].sens_data.acc.y) / accelScale;
        imuBuffer.accel.z = (bmi270Data[ACCEL].sens_data.acc.z) / accelScale;

        imuBuffer.gyro.x = (bmi270Data[GYRO].sens_data.gyr.x - gyroBias.x) * gyroValue2Degree;
        imuBuffer.gyro.y = (bmi270Data[GYRO].sens_data.gyr.y - gyroBias.y) * gyroValue2Degree;
        imuBuffer.gyro.z = (bmi270Data[GYRO].sens_data.gyr.z - gyroBias.z) * gyroValue2Degree;

        applyLpf(lpf2pAccel, &imuBuffer.accel);
        applyLpf(lpf2pGyro, &imuBuffer.gyro);

        STATIC_MUTEX_LOCK(imuDataMutex, portMAX_DELAY);
        imuData = imuBuffer;
        STATIC_MUTEX_UNLOCK(imuDataMutex);

        packet.imu = imuBuffer;
        estimatorKalmanEnqueue(&packet);

        // if (count++ % 250 == 0) {
        //     printf("IMU: %.3f, %.3f, %.3f | %.3f, %.3f, %.3f\n", 
        //         imuBuffer.accel.x, imuBuffer.accel.y, imuBuffer.accel.z,
        //         imuBuffer.gyro.x, imuBuffer.gyro.y, imuBuffer.gyro.z);
        // }
    }
}

void imuInit() {
    init_spi();
    STATIC_MUTEX_INIT(imuDataMutex);

    int8_t rslt;
    uint8_t chipId;
    bmi2Dev.delay_us = sensorsDelayUs;
    bmi2Dev.intf = BMI2_SPI_INTF;
    bmi2Dev.read = spiRead;
    bmi2Dev.write = spiWrite;
    bmi2Dev.read_write_len = 32;
    rslt = bmi270_init(&bmi2Dev);
    rslt |= bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chipId, 1, &bmi2Dev);
	if (rslt != BMI2_OK || chipId != BMI270_CHIP_ID) {
        printf("BMI270 Init [FAILED]: chipId: 0x%02x, rslt: %d\n", chipId, rslt);
        return;
    } else
        printf("BMI270 Init [OK]\n");

    bmi2Dev.delay_us(10000, NULL);
    bmi270Config[ACCEL].type = BMI2_ACCEL;
    bmi270Config[GYRO].type = BMI2_GYRO;
    uint8_t sensorsList[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sensorsList, 2, &bmi2Dev);
    /*! Disable power saving mode, this will cause severe delay */
    rslt |= bmi2_set_adv_power_save(BMI2_DISABLE, &bmi2Dev);
    if (rslt != BMI2_OK) {
        printf("BMI270 Enable [FAILED].\n");
        return;
    }
    bmi2Dev.delay_us(10000, NULL);

    bmi270Config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    bmi270Config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_16G;
    bmi270Config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    bmi270Config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    bmi270Config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;
    bmi270Config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    bmi270Config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    bmi270Config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    bmi270Config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    accelValue2Gravity = (float)16 / 32768.0f;
    gyroValue2Degree = (float)2000 / 32768.0f;

    rslt = bmi2_set_sensor_config(bmi270Config, 2, &bmi2Dev);
    if (rslt != BMI2_OK) {
        printf("BMI270 Accel Gyro Config [FAILED].\n");
        return;
    }
    bmi2Dev.delay_us(10000, NULL);

    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&lpf2pAccel[i], 1000, 20);
        lpf2pInit(&lpf2pGyro[i], 1000, 50);
    }

    xTaskCreatePinnedToCore(imuTask, "imu_task", 4096, NULL, 5, NULL, 1);
}
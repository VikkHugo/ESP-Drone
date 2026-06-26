/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Implements HAL for sensors MPU6050, HMC5883L and BMP180
 *
 * 2016.06.15: Initial version by Mike Hamer, http://mikehamer.info
 */
#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "queue.h"
#include "projdefs.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"

#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "stm32_legacy.h"

#include "i2cdev.h"
// #include "lps25h.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "ms5611.h"
// #include "ak8963.h"
#include "zranger.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "flowdeck_v1v2.h"
#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"
#include "static_mem.h"
#include "crtp_commander.h"

/**
 * Enable 250Hz digital LPF mode. However does not work with
 * multiple slave reading through MPU9250 (MAG and BARO), only single for some reason.
 */
//#define SENSORS_mpu6050_DLPF_256HZ

//#define GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES

#define MAG_GAUSS_PER_LSB 666.7f

/**
 * Enable sensors on board 
 */
#define SENSORS_ENABLE_MAG_HM5883L
#define SENSORS_ENABLE_PRESSURE_BMP180
//#define SENSORS_ENABLE_RANGE_VL53L0X
#define SENSORS_ENABLE_RANGE_VL53L1X
#define SENSORS_ENABLE_FLOW_PMW3901

#ifdef SENSORS_ENABLE_PRESSURE_BMP180
#define BMP180_I2C_ADDRESS_0       0x77
#define BMP180_I2C_ADDRESS_1       0x76
#define BMP180_REG_ID              0xD0
#define BMP180_REG_CALIB_START     0xAA
#define BMP180_CALIB_DATA_LENGTH   22
#define BMP180_REG_CONTROL         0xF4
#define BMP180_REG_RESULT          0xF6
#define BMP180_CMD_TEMP            0x2E
#define BMP180_CMD_PRESSURE        0x34

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp180_calib_t;

static bmp180_calib_t bmp180_calib;
static uint8_t bmp180_i2c_addr = BMP180_I2C_ADDRESS_0;
static int32_t bmp180_b5 = 0;

static bool bmp180ReadCalibration(I2C_Dev *i2c, uint8_t addr);
static bool bmp180Init(I2C_Dev *i2c);
static int32_t bmp180ReadRawTemp(I2C_Dev *i2c, uint8_t addr);
static int32_t bmp180ReadRawPressure(I2C_Dev *i2c, uint8_t addr, uint8_t oss);
static float bmp180CompensateTemperature(int32_t ut);
static float bmp180CompensatePressure(int32_t up, uint8_t oss);
#endif

#define SENSORS_GYRO_FS_CFG MPU6050_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG MPU6050_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG MPU6050_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG MPU6050_G_PER_LSB_16

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX 5.0f             // Max degrees off

#define SENSORS_BIAS_SAMPLES 1000
#define SENSORS_ACC_SCALE_SAMPLES 200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

// Buffer length for MPU9250 slave reads
#define GPIO_INTA_MPU6050_IO CONFIG_MPU_PIN_INT
#define SENSORS_MPU6050_BUFF_LEN 14
#define SENSORS_MAG_BUFF_LEN 8
#ifdef SENSORS_ENABLE_PRESSURE_BMP180
#define SENSORS_BARO_BUFF_S_P_LEN 0
#define SENSORS_BARO_BUFF_T_LEN 0
#else
#define SENSORS_BARO_BUFF_S_P_LEN MS5611_D1D2_SIZE
#define SENSORS_BARO_BUFF_T_LEN MS5611_D1D2_SIZE
#endif
#define SENSORS_BARO_BUFF_LEN (SENSORS_BARO_BUFF_S_P_LEN + SENSORS_BARO_BUFF_T_LEN)

#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)
// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE 5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
#define ESP_INTR_FLAG_DEFAULT 0

#define PITCH_CALIB (CONFIG_PITCH_CALIB*1.0/100)
#define ROLL_CALIB (CONFIG_ROLL_CALIB*1.0/100)

typedef struct {
    Axis3f bias;
    Axis3f variance;
    Axis3f mean;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined(GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
#ifdef SENSORS_ENABLE_RANGE_VL53L1X
static bool isVl53l1xPresent __attribute__((unused)) = false;
#endif
#ifdef SENSORS_ENABLE_RANGE_VL53L0X
static bool isVl53l0xPresent = false;
#endif
#ifdef SENSORS_ENABLE_FLOW_PMW3901
static bool isPmw3901Present = false;
#endif
static bool isMpu6050TestPassed = false;
#ifdef SENSORS_ENABLE_MAG_HM5883L
static bool isHmc5883lTestPassed = false;
#endif
#ifdef SENSORS_ENABLE_PRESSURE_BMP180
    static bool isBmp180TestPassed = false;
#endif

// Pre-calculated values for accelerometer alignment
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;

// This buffer needs to hold data from all sensors
static uint8_t buffer[SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static void processAccGyroMeasurements(const uint8_t *buffer);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void processBarometerMeasurements(const uint8_t *buffer);
static void sensorsDeviceInit(void);
static void sensorsSetupSlaveRead(void);

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsMpu6050Hmc5883lMs5611AreCalibrated()
{
    return gyroBiasFound;
}

static void sensorsTask(void *param)
{
    //TODO:
    systemWaitStart();
    vTaskDelay(M2T(200));
    DEBUG_PRINTD("xTaskCreate sensorsTask IN");
    sensorsSetupSlaveRead(); //
    DEBUG_PRINTD("xTaskCreate sensorsTask SetupSlave done");

    while (1) {

        /* mpu6050 interrupt trigger: data is ready to be read */
        if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
            sensorData.interruptTimestamp = imuIntTimestamp;

            /* sensors step 1-read data from I2C */
            uint8_t dataLen = (uint8_t)(SENSORS_MPU6050_BUFF_LEN +
                                        (isMagnetometerPresent ? SENSORS_MAG_BUFF_LEN : 0) +
                                        (isBarometerPresent ? SENSORS_BARO_BUFF_LEN : 0));
            i2cdevReadReg8(I2C0_DEV, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, dataLen, buffer);

            /* sensors step 2-process the respective data */
            processAccGyroMeasurements(&(buffer[0]));

            if (isMagnetometerPresent) {
                processMagnetometerMeasurements(&(buffer[SENSORS_MPU6050_BUFF_LEN]));
            }

            if (isBarometerPresent) {
                processBarometerMeasurements(&(buffer[isMagnetometerPresent ? SENSORS_MPU6050_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6050_BUFF_LEN]));
            }

            /* sensors step 3- queue sensors data  on the output queues */
            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);

            if (isMagnetometerPresent) {
                xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
            }

            if (isBarometerPresent) {
                xQueueOverwrite(barometerDataQueue, &sensorData.baro);
            }

            /* sensors step 4- Unlock stabilizer task */
            xSemaphoreGive(dataReady);
#ifdef DEBUG_EP2
            if (isBarometerPresent) {
                DEBUG_PRINT_LOCAL("ax=%f ay=%f az=%f gx=%f gy=%f gz=%f hx=%f hy=%f hz=%f pres=%f temp=%f asl=%f\n",
                    sensorData.acc.x, sensorData.acc.y, sensorData.acc.z,
                    sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z,
                    sensorData.mag.x, sensorData.mag.y, sensorData.mag.z,
                    sensorData.baro.pressure, sensorData.baro.temperature, sensorData.baro.asl);
            } else {
                DEBUG_PRINT_LOCAL("ax=%f ay=%f az=%f gx=%f gy=%f gz=%f hx=%f hy=%f hz=%f\n",
                    sensorData.acc.x, sensorData.acc.y, sensorData.acc.z,
                    sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z,
                    sensorData.mag.x, sensorData.mag.y, sensorData.mag.z);
            }
#endif
        }
    }
}

void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

void processBarometerMeasurements(const uint8_t *buffer)
{
#ifdef SENSORS_ENABLE_PRESSURE_BMP180
    // Read barometer via direct I2C reads (preferred) — using raw conversions
    int32_t ut = bmp180ReadRawTemp(I2C0_DEV, bmp180_i2c_addr);
    int32_t up = bmp180ReadRawPressure(I2C0_DEV, bmp180_i2c_addr, 0);
    float temperature = bmp180CompensateTemperature(ut);
    float pressurePa = bmp180CompensatePressure(up, 0);

    sensorData.baro.pressure = pressurePa / 100.0f; // Pa -> hPa/mbar
    sensorData.baro.temperature = temperature;
    sensorData.baro.asl = ms5611PressureToAltitude(&sensorData.baro.pressure);
#else
    DEBUG_PRINTW("processBarometerMeasurements NEED TODO");
#endif
}

static void sensorsDeviceInit(void)
{
    isMagnetometerPresent = false;
    isBarometerPresent = false;
#ifdef SENSORS_ENABLE_PRESSURE_BMP180
    isBmp180TestPassed = false;
#endif
#ifdef SENSORS_ENABLE_MAG_HM5883L
    isHmc5883lTestPassed = false;
#endif
    isMpu6050TestPassed = false;

    // Wait for sensors to startup
    while (xTaskGetTickCount() < 1000);

    i2cdevInit(I2C0_DEV);
    mpu6050Init(I2C0_DEV);

    if (mpu6050TestConnection() == true) {
        isMpu6050TestPassed = true;
        DEBUG_PRINTI("MPU6050 I2C connection [OK].\n");
    } else {
        DEBUG_PRINTW("MPU6050 I2C connection [FAIL].\n");
    }

    mpu6050Reset();
    vTaskDelay(M2T(50));

    mpu6050SetSleepEnabled(false);
    vTaskDelay(M2T(100));
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    vTaskDelay(M2T(200));
    mpu6050SetTempSensorEnabled(true);
    mpu6050SetIntEnabled(false);
    mpu6050SetI2CBypassEnabled(true);
    mpu6050SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);
    mpu6050SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);

#if SENSORS_MPU6050_DLPF_256HZ
    mpu6050SetRate(7);
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
#else
    mpu6050SetRate(0);
    /* Use MPU6050 CONFIG DLPF for ~42Hz bandwidth for both gyro/accel */
    mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }
#endif

#ifdef SENSORS_ENABLE_MAG_HM5883L
    hmc5883lInit(I2C0_DEV);
    if (hmc5883lTestConnection() == true) {
        isMagnetometerPresent = true;
        isHmc5883lTestPassed = true;
        hmc5883lSetMode(HMC5883L_MODE_CONTINUOUS);
        DEBUG_PRINTI("HMC5883L I2C connection [OK].\n");
    } else {
        DEBUG_PRINTW("HMC5883L I2C connection [FAIL].\n");
    }
#endif

#ifdef SENSORS_ENABLE_PRESSURE_BMP180
    if (bmp180Init(I2C0_DEV) == true) {
        isBarometerPresent = true;
        isBmp180TestPassed = true;
        DEBUG_PRINTI("BMP180 I2C connection [OK].\n");
    } else {
        DEBUG_PRINTW("BMP180 I2C connection [FAIL].\n");
    }
#endif

    DEBUG_PRINTI("GY-87 sensor probe -> MPU6050:%s HMC5883L:%s BMP180:%s\n",
                 isMpu6050TestPassed ? "OK" : "FAIL",
                 isMagnetometerPresent ? "OK" : "FAIL",
                 isBarometerPresent ? "OK" : "FAIL");
}

#ifdef SENSORS_ENABLE_PRESSURE_BMP180
static bool bmp180ReadCalibration(I2C_Dev *i2c, uint8_t addr)
{
    uint8_t calib[BMP180_CALIB_DATA_LENGTH];
    if (!i2cdevReadReg8(i2c, addr, BMP180_REG_CALIB_START, BMP180_CALIB_DATA_LENGTH, calib)) {
        return false;
    }

    bmp180_calib.ac1 = (int16_t)((calib[0] << 8) | calib[1]);
    bmp180_calib.ac2 = (int16_t)((calib[2] << 8) | calib[3]);
    bmp180_calib.ac3 = (int16_t)((calib[4] << 8) | calib[5]);
    bmp180_calib.ac4 = (uint16_t)((calib[6] << 8) | calib[7]);
    bmp180_calib.ac5 = (uint16_t)((calib[8] << 8) | calib[9]);
    bmp180_calib.ac6 = (uint16_t)((calib[10] << 8) | calib[11]);
    bmp180_calib.b1 = (int16_t)((calib[12] << 8) | calib[13]);
    bmp180_calib.b2 = (int16_t)((calib[14] << 8) | calib[15]);
    bmp180_calib.mb = (int16_t)((calib[16] << 8) | calib[17]);
    bmp180_calib.mc = (int16_t)((calib[18] << 8) | calib[19]);
    bmp180_calib.md = (int16_t)((calib[20] << 8) | calib[21]);

    return true;
}

static bool bmp180Init(I2C_Dev *i2c)
{
    uint8_t id = 0;
    uint8_t addresses[] = { BMP180_I2C_ADDRESS_0, BMP180_I2C_ADDRESS_1 };

    for (uint8_t i = 0; i < sizeof(addresses); i++) {
        uint8_t addr = addresses[i];
        if (!i2cdevReadByte(i2c, addr, BMP180_REG_ID, &id)) {
            continue;
        }

        if (id != 0x55) {
            continue;
        }

        if (!bmp180ReadCalibration(i2c, addr)) {
            continue;
        }

        bmp180_i2c_addr = addr;
        return true;
    }

    return false;
}

static int32_t bmp180ReadRawTemp(I2C_Dev *i2c, uint8_t addr)
{
    uint8_t cmd = BMP180_CMD_TEMP;
    if (!i2cdevWriteReg8(i2c, addr, BMP180_REG_CONTROL, 1, &cmd)) {
        return -1;
    }
    vTaskDelay(M2T(5));
    uint8_t data[2] = {0};
    if (!i2cdevReadReg8(i2c, addr, BMP180_REG_RESULT, 2, data)) {
        return -1;
    }
    return (int32_t)((data[0] << 8) | data[1]);
}

static int32_t bmp180ReadRawPressure(I2C_Dev *i2c, uint8_t addr, uint8_t oss)
{
    uint8_t cmd = BMP180_CMD_PRESSURE + (oss << 6);
    if (!i2cdevWriteReg8(i2c, addr, BMP180_REG_CONTROL, 1, &cmd)) {
        return -1;
    }
    vTaskDelay(M2T(5));
    uint8_t data[3] = {0};
    if (!i2cdevReadReg8(i2c, addr, BMP180_REG_RESULT, 3, data)) {
        return -1;
    }
    int32_t up = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    up = up >> (8 - oss);
    return up;
}

static float bmp180CompensateTemperature(int32_t ut)
{
    int32_t x1 = (((int32_t)ut - (int32_t)bmp180_calib.ac6) * (int32_t)bmp180_calib.ac5) >> 15;
    int32_t x2 = ((int32_t)bmp180_calib.mc << 11) / (x1 + bmp180_calib.md);
    bmp180_b5 = x1 + x2;
    int32_t t = (bmp180_b5 + 8) >> 4; // temp in 0.1 C
    return ((float)t) / 10.0f;
}

static float bmp180CompensatePressure(int32_t up, uint8_t oss)
{
    int32_t b6 = bmp180_b5 - 4000;
    int64_t x1 = ((int64_t)bmp180_calib.b2 * ((b6 * b6) >> 12)) >> 11;
    int64_t x2 = ((int64_t)bmp180_calib.ac2 * b6) >> 11;
    int64_t x3 = x1 + x2;
    int64_t b3 = ((((int64_t)bmp180_calib.ac1 * 4 + x3) << oss) + 2) >> 2;
    x1 = ((int64_t)bmp180_calib.ac3 * b6) >> 13;
    x2 = ((int64_t)bmp180_calib.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint64_t b4 = ((uint64_t)bmp180_calib.ac4 * (uint64_t)(x3 + 32768)) >> 15;
    uint64_t b7 = ((uint64_t)up - b3) * (50000 >> oss);
    int64_t p;
    if (b7 < 0x80000000ULL) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    return (float)p; // pressure in Pa
}
#endif

void processMagnetometerMeasurements(const uint8_t *buffer)
{
    if (buffer[7] & (1 << HMC5883L_STATUS_READY_BIT)) {
        int16_t headingx = (((int16_t)buffer[2]) << 8) | buffer[1]; //hmc5883 different from
        int16_t headingz = (((int16_t)buffer[4]) << 8) | buffer[3];
        int16_t headingy = (((int16_t)buffer[6]) << 8) | buffer[5];

        sensorData.mag.x = (float)headingx / MAG_GAUSS_PER_LSB; //to gauss
        sensorData.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
        sensorData.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
    }
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
    /*  Note the ordering to correct the rotated 90º IMU coordinate system */

    Axis3f accScaled;

#ifdef CONFIG_TARGET_ESPLANE_V1
    /* sensors step 2.1 read from buffer */
    /*
    accelRaw.x = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.y = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.x = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.y = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
    */
    accelRaw.y = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.x = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.y = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.x = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
#else
    /* sensors step 2.1 read from buffer */
    accelRaw.y = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.x = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.y = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.x = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
#endif

#ifdef GYRO_BIAS_LIGHT_WEIGHT
    gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
    /* sensors step 2.2 Calculates the gyro bias first when the  variance is below threshold */
    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif

    if (gyroBiasFound) {
        processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
    }

    sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.y =  (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.z =  (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
    applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

    accScaled.x = -(accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.y =  (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.z =  (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;
    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
    applyAxis3fLpf((lpf2pData*)(&accLpf), &sensorData.acc);
}

static void sensorsSetupSlaveRead(void)
{
    // Now begin to set up the slaves
#ifdef SENSORS_MPU6050_DLPF_256HZ
    // As noted in registersheet 4.4: "Data should be sampled at or above sample rate;
    // SMPLRT_DIV is only used for 1kHz internal sampling." Slowest update rate is then 500Hz.
    mpu6050SetSlave4MasterDelay(15); // read slaves at 500Hz = (8000Hz / (1 + 15))
#else
    mpu6050SetSlave4MasterDelay(9); // read slaves at 100Hz = (500Hz / (1 + 4))
#endif

    mpu6050SetI2CBypassEnabled(false);
    mpu6050SetWaitForExternalSensorEnabled(true);     // the slave data isn't so important for the state estimation
    mpu6050SetInterruptMode(0);                       // active high
    mpu6050SetInterruptDrive(0);                      // push pull
    mpu6050SetInterruptLatch(0);                      // latched until clear
    mpu6050SetInterruptLatchClear(1);                 // cleared on any register read
    mpu6050SetSlaveReadWriteTransitionEnabled(false); // Send a stop at the end of a slave read
    mpu6050SetMasterClockSpeed(13);                   // Set i2c speed to 400kHz

#ifdef SENSORS_ENABLE_MAG_HM5883L

    if (isMagnetometerPresent) {
        // Set registers for mpu6050 master to read from
        mpu6050SetSlaveAddress(0, 0x80 | HMC5883L_ADDRESS);        // set the magnetometer to Slave 0, enable read
        mpu6050SetSlaveRegister(0, HMC5883L_RA_MODE);       // read the magnetometer heading register
        mpu6050SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); // hmc5883l:model,x,z,y,status ak8963:read 8 bytes (ST1, x, y, z heading, ST2 (overflow check))
        mpu6050SetSlaveDelayEnabled(0, true);
        mpu6050SetSlaveEnabled(0, true);
        DEBUG_PRINTD("mpu6050SetSlaveAddress HMC5883L done \n");
    }

#endif

#ifdef SENSORS_ENABLE_PRESSURE_BMP180
    // BMP180 accessed directly via I2C; no MPU6050 slave config required
#endif

    // Enable sensors after configuration
    mpu6050SetI2CMasterModeEnabled(true);

    mpu6050SetIntDataReadyEnabled(true);

    DEBUG_PRINTD("sensorsSetupSlaveRead done \n");
}

static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
  DEBUG_PRINTD("xTaskCreate sensorsTask \n");
}

static void IRAM_ATTR sensors_inta_isr_handler(void *arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp(); //This function returns the number of microseconds since esp_timer was initialized
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void sensorsInterruptInit(void)
{

    DEBUG_PRINTD("sensorsInterruptInit \n");
    gpio_config_t io_conf = {
        //interrupt of rising edge
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_POSEDGE,
#else
        .intr_type = GPIO_PIN_INTR_POSEDGE,
#endif
        //bit mask of the pins
        .pin_bit_mask = (1ULL << GPIO_INTA_MPU6050_IO),
        //set as input mode
        .mode = GPIO_MODE_INPUT,
        //disable pull-down mode
        .pull_down_en = 0,
        //enable pull-up mode
        .pull_up_en = 1,
    };
    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    //install gpio isr service
    //portDISABLE_INTERRUPTS();
    gpio_set_intr_type(GPIO_INTA_MPU6050_IO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INTA_MPU6050_IO, sensors_inta_isr_handler, (void *)GPIO_INTA_MPU6050_IO);
    //portENABLE_INTERRUPTS();
    DEBUG_PRINTD("sensorsInterruptInit done \n");

    //   FSYNC "shall not be floating, must be set high or low by the MCU"

}

void sensorsMpu6050Hmc5883lMs5611Init(void)
{
    if (isInit) {
        return;
    }

    sensorsBiasObjInit(&gyroBiasRunning);
    sensorsDeviceInit();
    sensorsInterruptInit();
    sensorsTaskInit();
    isInit = true;
}

bool sensorsMpu6050Hmc5883lMs5611Test(void)
{

    bool testStatus = true;

    if (!isInit) {
        DEBUG_PRINTE("Error while initializing sensor task\r\n");
        testStatus = false;
    }

    // Try for 3 seconds so the quad has stabilized enough to pass the test
    for (int i = 0; i < 300; i++) {
        if (mpu6050SelfTest() == true) {
            isMpu6050TestPassed = true;
            break;
        } else {
            vTaskDelay(M2T(10));
        }
    }

    testStatus &= isMpu6050TestPassed;

#ifdef SENSORS_ENABLE_MAG_HM5883L
    testStatus &= isMagnetometerPresent;

    if (testStatus) {
        isHmc5883lTestPassed = hmc5883lSelfTest();
        testStatus &= isHmc5883lTestPassed;
    }

#endif

#ifdef SENSORS_ENABLE_PRESSURE_BMP180
    testStatus &= isBarometerPresent;

    if (testStatus) {
        testStatus &= isBmp180TestPassed;
    }

#endif

    return testStatus;

}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    static bool accBiasFound = false;
    static uint32_t accScaleSumCount = 0;

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
            accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
            accBiasFound = true;
        }
    }

    return accBiasFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    static uint32_t gyroBiasSampleCount = 0;
    static bool gyroBiasNoBuffFound = false;
    static Axis3i64 gyroBiasSampleSum;
    static Axis3i64 gyroBiasSampleSumSquares;

    if (!gyroBiasNoBuffFound) {
        // If the gyro has not yet been calibrated:
        // Add the current sample to the running mean and variance
        gyroBiasSampleSum.x += gx;
        gyroBiasSampleSum.y += gy;
        gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
        gyroBiasSampleSumSquares.x += gx * gx;
        gyroBiasSampleSumSquares.y += gy * gy;
        gyroBiasSampleSumSquares.z += gz * gz;
#endif
        gyroBiasSampleCount += 1;

        // If we then have enough samples, calculate the mean and standard deviation
        if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES) {
            gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
            gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
            gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
            gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
            gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
            gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
            gyroBiasNoBuffFound = true;
        }
    }

    return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound) {
        sensorsFindBiasValue(&gyroBiasRunning);

        if (gyroBiasRunning.isBiasValueFound) {
            //TODO:
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
            DEBUG_PRINTI("isBiasValueFound!");
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int64_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj *bias, Axis3i32 *meanOut)
{
    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
    }

    meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
                bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
                bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
                (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
            varianceSampleTime = xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
        }
    }

    return foundBias;
}

bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void)
{
    bool testStatus = false;
    Axis3i16 g;
    Axis3i16 a;
    Axis3f acc; // Accelerometer axis data in mG
    float pitch, roll;
    uint32_t startTick = xTaskGetTickCount();

    testStatus = mpu6050SelfTest();

    if (testStatus) {
        sensorsBiasObjInit(&gyroBiasRunning);

        while (xTaskGetTickCount() - startTick < SENSORS_VARIANCE_MAN_TEST_TIMEOUT) {
            mpu6050GetMotion6(&a.y, &a.x, &a.z, &g.y, &g.x, &g.z);

            if (processGyroBias(g.x, g.y, g.z, &gyroBias)) {
                gyroBiasFound = true;
                DEBUG_PRINTI("Gyro variance test [OK]\n");
                break;
            }
        }

        if (gyroBiasFound) {
            acc.x = (a.x) * SENSORS_G_PER_LSB_CFG;
            acc.y = (a.y) * SENSORS_G_PER_LSB_CFG;
            acc.z = (a.z) * SENSORS_G_PER_LSB_CFG;

            // Calculate pitch and roll based on accelerometer. Board must be level
            pitch = tanf(-acc.x / (sqrtf(acc.y * acc.y + acc.z * acc.z))) * 180 / (float)M_PI;
            roll = tanf(acc.y / acc.z) * 180 / (float)M_PI;

            if ((fabsf(roll) < SENSORS_MAN_TEST_LEVEL_MAX) && (fabsf(pitch) < SENSORS_MAN_TEST_LEVEL_MAX)) {
                DEBUG_PRINTI("Acc level test [OK]\n");
                testStatus = true;
            } else {
                DEBUG_PRINTE("Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", (double)roll, (double)pitch);
                testStatus = false;
            }
        } else {
            DEBUG_PRINTE("Gyro variance test [FAIL]\n");
            testStatus = false;
        }
    }

    return testStatus;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    Axis3f rx;
    Axis3f ry;

    // Rotate around x-axis
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around y-axis
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}

/** set different low pass filters in different environment
 *
 *
 */
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode)
{
    switch (accMode)
    {
    case ACC_MODE_PROPTEST:
        mpu6050SetRate(7);
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
        for (uint8_t i = 0; i < 3; i++)
        {
            lpf2pInit(&accLpf[i], 1000, 250);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        mpu6050SetRate(0);
#ifdef CONFIG_TARGET_ESP32_S2_DRONE_V1_2
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_42);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#else
        mpu6050SetDLPFMode(MPU6050_DLPF_BW_98);
        for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
#endif
        break;
    }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++) {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}

#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif

//TODO:
PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, mpu6050, &isMpu6050TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, HMC5883L, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, pmw3901, &isPmw3901Present)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MS5611, &isBarometerPresent) // TODO: Rename MS5611 to LPS25H. Client needs to be updated at the same time.
PARAM_GROUP_STOP(imu_tests)

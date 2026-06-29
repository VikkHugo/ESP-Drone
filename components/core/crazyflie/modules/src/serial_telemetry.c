#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queue.h"
#include "driver/uart.h"

#include "config.h"
#include "serial_telemetry.h"
#include "static_mem.h"
#define DEBUG_MODULE "SERIAL_TLM"
#include "debug_cf.h"

#define SERIAL_TELEMETRY_UART_NUM UART_NUM_1
#define SERIAL_TELEMETRY_UART_BAUDRATE 115200
#define SERIAL_TELEMETRY_QUEUE_LEN 1
#define SERIAL_TELEMETRY_TX_BUFFER_SIZE 1024
#define SERIAL_TELEMETRY_RX_BUFFER_SIZE 256
#define SERIAL_TELEMETRY_LINE_MAX_LEN 320
#define SERIAL_TELEMETRY_TASK_STACKSIZE (6 * configBASE_STACK_SIZE)

#ifndef SERIAL_TELEMETRY_UART_TX_PIN
#define SERIAL_TELEMETRY_UART_TX_PIN 16
#endif

#ifndef SERIAL_TELEMETRY_UART_RX_PIN
#define SERIAL_TELEMETRY_UART_RX_PIN 17
#endif

typedef struct {
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float mag_x;
  float mag_y;
  float mag_z;
  float altitude;
  float pressao;
  float roll;
  float pitch;
  float yaw;
} serialTelemetryFrame_t;

static bool isInit = false;
static xQueueHandle telemetryQueue;
STATIC_MEM_QUEUE_ALLOC(telemetryQueue, SERIAL_TELEMETRY_QUEUE_LEN, sizeof(serialTelemetryFrame_t));
STATIC_MEM_TASK_ALLOC(serialTelemetryTask, SERIAL_TELEMETRY_TASK_STACKSIZE);

// Keep the output buffer out of task stack to minimize stack pressure.
static char telemetryLine[SERIAL_TELEMETRY_LINE_MAX_LEN];

static void serialTelemetryTask(void* param)
{
  (void)param;
  serialTelemetryFrame_t frame;

  while (true) {
    if (xQueueReceive(telemetryQueue, &frame, portMAX_DELAY) == pdTRUE) {
      int written = snprintf(
          telemetryLine,
          sizeof(telemetryLine),
          "acc_x:%0.4f:acc_y:%0.4f:acc_z:%0.4f:gyro_x:%0.4f:gyro_y:%0.4f:gyro_z:%0.4f:mag_x:%0.4f:mag_y:%0.4f:mag_z:%0.4f:altitude:%0.4f:pressao:%0.4f:roll:%0.4f:pitch:%0.4f:yaw:%0.4f\n",
          (double)frame.acc_x, (double)frame.acc_y, (double)frame.acc_z,
          (double)frame.gyro_x, (double)frame.gyro_y, (double)frame.gyro_z,
          (double)frame.mag_x, (double)frame.mag_y, (double)frame.mag_z,
          (double)frame.altitude, (double)frame.pressao,
          (double)frame.roll, (double)frame.pitch, (double)frame.yaw);

      if (written > 0) {
        int len = written;
        if (len >= (int)sizeof(telemetryLine)) {
          len = sizeof(telemetryLine) - 1;
        }
        uart_write_bytes(SERIAL_TELEMETRY_UART_NUM, telemetryLine, len);
      }
    }
  }
}

void serialTelemetryInit(void)
{
  if (isInit) {
    return;
  }

  const uart_config_t uartConfig = {
      .baud_rate = SERIAL_TELEMETRY_UART_BAUDRATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t err = uart_driver_install(
      SERIAL_TELEMETRY_UART_NUM,
      SERIAL_TELEMETRY_RX_BUFFER_SIZE,
      SERIAL_TELEMETRY_TX_BUFFER_SIZE,
      0,
      NULL,
      0);

  if (err != ESP_OK) {
    DEBUG_PRINTW("uart_driver_install failed: %d", err);
    return;
  }

  err = uart_param_config(SERIAL_TELEMETRY_UART_NUM, &uartConfig);
  if (err != ESP_OK) {
    DEBUG_PRINTW("uart_param_config failed: %d", err);
    return;
  }

  err = uart_set_pin(
      SERIAL_TELEMETRY_UART_NUM,
      SERIAL_TELEMETRY_UART_TX_PIN,
      SERIAL_TELEMETRY_UART_RX_PIN,
      UART_PIN_NO_CHANGE,
      UART_PIN_NO_CHANGE);

  if (err != ESP_OK) {
    DEBUG_PRINTW("uart_set_pin failed: %d", err);
    return;
  }

  telemetryQueue = STATIC_MEM_QUEUE_CREATE(telemetryQueue);
  STATIC_MEM_TASK_CREATE(serialTelemetryTask, serialTelemetryTask, "SERIAL_TLM", NULL, 1);

  isInit = true;
  DEBUG_PRINTI("Serial telemetry ready on UART1 TX=%d RX=%d", SERIAL_TELEMETRY_UART_TX_PIN, SERIAL_TELEMETRY_UART_RX_PIN);
}

bool serialTelemetryIsInit(void)
{
  return isInit;
}

void serialTelemetryPush(const sensorData_t* sensorData, const state_t* state)
{
  if (!isInit || sensorData == NULL || state == NULL) {
    return;
  }

  serialTelemetryFrame_t frame = {
      .acc_x = sensorData->acc.x,
      .acc_y = sensorData->acc.y,
      .acc_z = sensorData->acc.z,
      .gyro_x = sensorData->gyro.x,
      .gyro_y = sensorData->gyro.y,
      .gyro_z = sensorData->gyro.z,
      .mag_x = sensorData->mag.x,
      .mag_y = sensorData->mag.y,
      .mag_z = sensorData->mag.z,
      .altitude = sensorData->baro.asl,
      .pressao = sensorData->baro.pressure,
      .roll = state->attitude.roll,
      .pitch = state->attitude.pitch,
      .yaw = state->attitude.yaw,
  };

  xQueueOverwrite(telemetryQueue, &frame);
}

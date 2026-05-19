/* BLE 模块实现：
 * - USART2 中断方式逐字节接收速度命令；
 * - 将解析出的 m/s 目标速度写入电机模块；
 * - FreeRTOS 任务周期发送当前速度反馈。
 */
#include "BLE.h"

#include <stdio.h>
#include <stdlib.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "motor.h"
#include "task.h"
#include "usart.h"

#define BLE_TELEMETRY_PERIOD_MS 10U
#define BLE_TX_BUFFER_SIZE      24U

typedef struct
{
  /* 支持形如 "-0.35\r\n" 的简单十进制速度命令，避免在中断里使用 sscanf。 */
  int32_t integer_part;
  int32_t fractional_part;
  int32_t fractional_scale;
  int8_t sign;
  uint8_t has_digit;
  uint8_t decimal_seen;
} BLE_RxParser_t;

static osThreadId_t bleTaskHandle = NULL;
static const osThreadAttr_t bleTaskAttributes = {
  .name = "bleTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

static uint8_t ble_rx_byte = 0U;
static BLE_RxParser_t ble_rx_parser = {0};

static void BLE_Task(void *argument);

/* 清空当前命令解析状态，下一字节从新命令开始解析。 */
static void BLE_ResetParser(void)
{
  ble_rx_parser.integer_part = 0;
  ble_rx_parser.fractional_part = 0;
  ble_rx_parser.fractional_scale = 1;
  ble_rx_parser.sign = 1;
  ble_rx_parser.has_digit = 0U;
  ble_rx_parser.decimal_seen = 0U;
}

static void BLE_ArmReception(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &ble_rx_byte, 1U);
}

static void BLE_CommitParserValue(void)
{
  float parsed_value;

  if (ble_rx_parser.has_digit == 0U)
  {
    BLE_ResetParser();
    return;
  }

  parsed_value = (float)ble_rx_parser.integer_part;

  if (ble_rx_parser.fractional_scale > 1)
  {
    parsed_value += ((float)ble_rx_parser.fractional_part / (float)ble_rx_parser.fractional_scale);
  }

  Motor_SetTargetSpeedMps(parsed_value * (float)ble_rx_parser.sign);
  BLE_ResetParser();
}

static void BLE_ProcessReceivedByte(uint8_t data)
{
  if ((data == '\r') || (data == '\n') || (data == ',') || (data == ';'))
  {
    BLE_CommitParserValue();
    return;
  }

  if ((data == ' ') || (data == '\t'))
  {
    return;
  }

  if ((data == '+') || (data == '-'))
  {
    if ((ble_rx_parser.has_digit == 0U) &&
        (ble_rx_parser.decimal_seen == 0U) &&
        (ble_rx_parser.integer_part == 0) &&
        (ble_rx_parser.fractional_part == 0))
    {
      ble_rx_parser.sign = (data == '-') ? -1 : 1;
      return;
    }

    BLE_ResetParser();
    return;
  }

  if (data == '.')
  {
    if (ble_rx_parser.decimal_seen == 0U)
    {
      ble_rx_parser.decimal_seen = 1U;
      return;
    }

    BLE_ResetParser();
    return;
  }

  if ((data >= '0') && (data <= '9'))
  {
    ble_rx_parser.has_digit = 1U;

    if (ble_rx_parser.decimal_seen == 0U)
    {
      ble_rx_parser.integer_part = (ble_rx_parser.integer_part * 10) + (int32_t)(data - '0');
    }
    else if (ble_rx_parser.fractional_scale < 1000)
    {
      ble_rx_parser.fractional_part = (ble_rx_parser.fractional_part * 10) + (int32_t)(data - '0');
      ble_rx_parser.fractional_scale *= 10;
    }

    return;
  }

  BLE_ResetParser();
}

static uint16_t BLE_FormatTelemetry(float speed_mps, char *buffer, uint16_t buffer_size)
{
  int32_t scaled_speed = (int32_t)(speed_mps * 100.0f);
  int32_t absolute_speed;
  int32_t integer_part;
  int32_t fractional_part;
  int written_length;

  if (speed_mps >= 0.0f)
  {
    scaled_speed = (int32_t)(speed_mps * 100.0f + 0.5f);
  }
  else
  {
    scaled_speed = (int32_t)(speed_mps * 100.0f - 0.5f);
  }

  absolute_speed = abs(scaled_speed);
  integer_part = absolute_speed / 100;
  fractional_part = absolute_speed % 100;

  if (scaled_speed < 0)
  {
    written_length = snprintf(buffer, buffer_size, "-%ld.%02ld\r\n", (long)integer_part, (long)fractional_part);
  }
  else
  {
    written_length = snprintf(buffer, buffer_size, "%ld.%02ld\r\n", (long)integer_part, (long)fractional_part);
  }

  if (written_length < 0)
  {
    return 0U;
  }

  if (written_length > (int)buffer_size)
  {
    return buffer_size;
  }

  return (uint16_t)written_length;
}

void BLE_Init(void)
{
  BLE_ResetParser();
  BLE_ArmReception();
}

void BLE_StartTask(void)
{
  if (bleTaskHandle == NULL)
  {
    bleTaskHandle = osThreadNew(BLE_Task, NULL, &bleTaskAttributes);
  }
}

void BLE_RxCpltCallbackFromISR(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    BLE_ProcessReceivedByte(ble_rx_byte);
    BLE_ArmReception();
  }
}

void BLE_ErrorCallbackFromISR(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    BLE_ResetParser();
    BLE_ArmReception();
  }
}

static void BLE_Task(void *argument)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  char tx_buffer[BLE_TX_BUFFER_SIZE];
  uint16_t tx_length;

  (void)argument;

  for (;;)
  {
    tx_length = BLE_FormatTelemetry(Motor_GetCurrentSpeedMps(), tx_buffer, sizeof(tx_buffer));
    if (tx_length > 0U)
    {
      (void)HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, tx_length, 20U);
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BLE_TELEMETRY_PERIOD_MS));
  }
}

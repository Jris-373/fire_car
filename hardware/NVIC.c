/* NVIC 模块实现：
 * 集中管理当前工程使用的中断优先级，并把 HAL 回调入口统一分发到业务模块。
 */
#include "NVIC.h"

#include "BLE.h"
#include "ESP01S.h"
#include "motor.h"

/* FreeRTOS 允许调用 FromISR API 的最高抢占优先级为 5，USART 使用该等级。 */
#define HARDWARE_NVIC_USART_PRIORITY     5U
#define HARDWARE_NVIC_KERNEL_PRIORITY    15U
#define HARDWARE_NVIC_DEFAULT_SUBPRIO    0U

extern volatile float g_left_motor_speed_mps;
extern volatile float g_right_motor_speed_mps;

/* TIM6 是 HAL tick 时基，优先级跟随 HAL_InitTick() 的 TickPriority 参数。 */
HAL_StatusTypeDef Hardware_NVIC_InitHALTimebaseIRQ(uint32_t tick_priority)
{
  if (tick_priority >= (1UL << __NVIC_PRIO_BITS))
  {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, tick_priority, HARDWARE_NVIC_DEFAULT_SUBPRIO);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  return HAL_OK;
}

void Hardware_NVIC_InitKernelIRQ(void)
{
  HAL_NVIC_SetPriority(PendSV_IRQn, HARDWARE_NVIC_KERNEL_PRIORITY, HARDWARE_NVIC_DEFAULT_SUBPRIO);
}

void Hardware_NVIC_InitPeripheralIRQs(void)
{
  HAL_NVIC_SetPriority(UART4_IRQn, HARDWARE_NVIC_USART_PRIORITY, HARDWARE_NVIC_DEFAULT_SUBPRIO);
  HAL_NVIC_EnableIRQ(UART4_IRQn);

  HAL_NVIC_SetPriority(USART2_IRQn, HARDWARE_NVIC_USART_PRIORITY, HARDWARE_NVIC_DEFAULT_SUBPRIO);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void Hardware_NVIC_DeInitPeripheralIRQs(void)
{
  HAL_NVIC_DisableIRQ(UART4_IRQn);
  HAL_NVIC_DisableIRQ(USART2_IRQn);
}

void Hardware_NVIC_DeInitUART4IRQ(void)
{
  HAL_NVIC_DisableIRQ(UART4_IRQn);
}

void Hardware_NVIC_DeInitUSART2IRQ(void)
{
  HAL_NVIC_DisableIRQ(USART2_IRQn);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART2)
  {
    /* BLE 使用 USART2 单字节中断接收。 */
    BLE_RxCpltCallbackFromISR(huart);
  }
  else if (huart->Instance == UART4)
  {
    /* ESP-01S 使用 UART4 单字节中断接收 AT/MQTT 响应。 */
    ESP01S_RxCpltCallbackFromISR(huart);
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART2)
  {
    /* USART2 错误后重新武装 BLE 的单字节接收。 */
    BLE_ErrorCallbackFromISR(huart);
  }
  else if (huart->Instance == UART4)
  {
    /* UART4 错误后重新武装 ESP-01S 的单字节接收。 */
    ESP01S_ErrorCallbackFromISR(huart);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == NULL)
  {
    return;
  }

  if (htim->Instance == TIM6)
  {
    /* HAL 系统 tick，保持 HAL_Delay 和 HAL_GetTick 正常。 */
    HAL_IncTick();
  }
  else if (htim->Instance == TIM10)
  {
    /* TIM10 驱动 500 Hz 电机速度环，并同步调试观察变量。 */
    Motor_SpeedLoopISR();
    g_left_motor_speed_mps = Motor_GetCurrentSpeedMpsForMotor(Motor_GetLeft());
    g_right_motor_speed_mps = Motor_GetCurrentSpeedMpsForMotor(Motor_GetRight());
  }
}

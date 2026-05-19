/* BLE 模块：通过 USART2 接收上位机/蓝牙速度指令，并周期发送速度反馈。 */
#ifndef __BLE_H__
#define __BLE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 初始化 USART2 单字节中断接收状态。 */
void BLE_Init(void);

/* 创建 BLE 遥测发送任务。 */
void BLE_StartTask(void);

/* 由 NVIC.c 中的 HAL UART 回调分发调用，运行在中断上下文。 */
void BLE_RxCpltCallbackFromISR(UART_HandleTypeDef *huart);
void BLE_ErrorCallbackFromISR(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_H__ */

/* NVIC 模块：集中登记外设中断优先级，并承载 HAL 中断回调分发入口。 */
#ifndef __HARDWARE_NVIC_H__
#define __HARDWARE_NVIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* HAL TIM6 时基中断，TickPriority 由 HAL_InitTick() 传入。 */
HAL_StatusTypeDef Hardware_NVIC_InitHALTimebaseIRQ(uint32_t tick_priority);

/* FreeRTOS 内核相关中断优先级配置。 */
void Hardware_NVIC_InitKernelIRQ(void);

/* 应用外设中断登记：UART4 和 USART2。 */
void Hardware_NVIC_InitPeripheralIRQs(void);

/* 应用外设中断反初始化接口。 */
void Hardware_NVIC_DeInitPeripheralIRQs(void);
void Hardware_NVIC_DeInitUART4IRQ(void);
void Hardware_NVIC_DeInitUSART2IRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_NVIC_H__ */

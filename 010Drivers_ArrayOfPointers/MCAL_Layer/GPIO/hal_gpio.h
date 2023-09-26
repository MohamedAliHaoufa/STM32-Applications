/*
 * hal_gpio.h
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_
#include "hal_gpio_reg.h"
#include "hal_gpio_cfg.h"

Std_ReturnType EXTI_EnableDisableInterrupt(uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType GPIO_InitPin(GPIO_PinConfig_t GpioPinConfig);
//Std_ReturnType GPIO_DeInitPort(GPIO_PortNumIndexArr_t PortNum);
uint8_t 	   GPIO_ReadInputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
uint16_t       GPIO_ReadInputPort(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
Std_ReturnType GPIO_SetResetOutputPin( GPIO_PortNumIndexArr_t PortNum, uint8_t value );
Std_ReturnType GPIO_ToggleOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
Std_ReturnType GPIO_WriteOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum, uint8_t value );
Std_ReturnType GPIO_WriteOutputPort(GPIO_PortNumIndexArr_t PortNum, uint16_t value);
Std_ReturnType GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
Std_ReturnType GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQpriority);
Std_ReturnType GPIO_IRQHandling(uint16_t PinNum);

void Delay(void);
void Delay_Btn(void);

#endif /* HAL_GPIO_H_ */

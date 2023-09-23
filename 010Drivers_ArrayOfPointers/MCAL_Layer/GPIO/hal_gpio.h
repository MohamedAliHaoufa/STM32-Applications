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

Std_ReturnType GPIO_Init(GPIO_PinConfig_t GpioPinConfig);
Std_ReturnType GPIO_DeInit(GPIO_PortNumIndexArr_t PortNum);
Std_ReturnType GPIO_ODRSetReset( GPIO_PortNumIndexArr_t PortNum, uint8_t value );
uint8_t GPIO_ReadFromInputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
Std_ReturnType GPIO_ToggleOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
Std_ReturnType GPIO_ReadFromOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum, uint8_t value );
Std_ReturnType GPIO_ReadFromOutputPort(GPIO_PortNumIndexArr_t PortNum, uint16_t value);
Std_ReturnType GPIO_IRQinterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
Std_ReturnType GPIO_IRQperiorityConfig(uint8_t IRQNumber, uint32_t IRQpriority);
Std_ReturnType GPIO_IRQHandling(uint16_t PinNum);

void delay(void);

#endif /* HAL_GPIO_H_ */

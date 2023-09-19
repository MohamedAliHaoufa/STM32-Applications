/*
 * GPIO.h
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#ifndef GPIO_H_
#define GPIO_H_
#include "GPIO_config.h"
#include "GPIO_reg.h"
uint8_t GPIO_ReadFromInputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
Std_ReturnType GPIO_ToggleOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum);
Std_ReturnType GPIO_ReadFromOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum, uint8_t value );
Std_ReturnType GPIO_ReadFromOutputPort(GPIO_PortNumIndexArr_t PortNum, uint16_t value);
Std_ReturnType GPIO_PeripheralClockControl (GPIO_PortNumIndexArr_t PortNum, uint8_t EnorDi);

void delay(void);
extern volatile RCC_AHB1ENR_Register *const RCC_Port_AHB1ClockEnable_Registers[Port_Indices];
extern volatile GPIO_MODER_Register *const GPIO_ModeSelect_Registers[Port_Indices];
extern volatile GPIO_ODR_Register *const GPIO_ODRControl_Registers[Port_Indices];
extern volatile RCC_CFGR_Register *const RCC_Port_ClockConfig_Register[Port_Indices];

#endif /* GPIO_H_ */

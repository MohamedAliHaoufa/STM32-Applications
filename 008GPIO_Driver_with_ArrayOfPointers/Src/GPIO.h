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

void delay(void);
extern volatile RCC_AHB1ENR_Register *const RCC_Port_AHB1ClockEnable_Registers[Port_Indices];
extern volatile GPIO_MODER_Register *const GPIO_ModeSelect_Registers[Port_Indices];
extern volatile GPIO_ODR_Register *const GPIO_ODRControl_Registers[Port_Indices];
extern volatile RCC_CFGR_Register *const RCC_Port_ClockConfig_Register[Port_Indices];

#endif /* GPIO_H_ */

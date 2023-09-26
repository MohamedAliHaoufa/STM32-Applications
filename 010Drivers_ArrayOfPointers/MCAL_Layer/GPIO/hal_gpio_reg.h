/*
 * hal_gpio_reg.h
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#ifndef HAL_GPIO_REG_H_
#define HAL_GPIO_REG_H_

#include "../mcal_stm32f407xx.h"

/**************************************************************************************
 **************************** Macros Definition ***************************************
 **************************************************************************************/

#define GPIO_PortA_RegisterControl GPIOA
#define GPIO_PortB_RegisterControl GPIOB
#define GPIO_PortC_RegisterControl GPIOC
#define GPIO_PortD_RegisterControl GPIOD
#define GPIO_PortE_RegisterControl GPIOE
#define GPIO_PortF_RegisterControl GPIOF

#define EXTI_RegisterControl EXTI
#define SYSCFG_RegisterControl SYSCFG

#endif /* HAL_GPIO_REG_H_ */

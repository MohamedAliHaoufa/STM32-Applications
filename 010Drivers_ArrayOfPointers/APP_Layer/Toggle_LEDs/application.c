/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

#include "../MCAL_Layer/mcal_stm32f407xx.h"


int main(void)
{
	// if array of pointers are static , you need to include GPIO.c
	// and if you remove static, include GPIO.h only, and extern them from GPIO.h
	Std_ReturnType k = E_OK;

	GPIO_PinConfig_t GpioPinConfig;

	GpioPinConfig.GPIO_PortIndex= PortD;
	GpioPinConfig.GPIO_PinNumber= Pin_No_12;
	GpioPinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	GpioPinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioPinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	GpioPinConfig.GPIO_PinPinOPType= GPIO_OP_TYPE_PP;

	GPIO_InitPin(GpioPinConfig);
	GpioPinConfig.GPIO_PinNumber= Pin_No_13;
	GPIO_InitPin(GpioPinConfig);
	GpioPinConfig.GPIO_PinNumber= Pin_No_14;
	GPIO_InitPin(GpioPinConfig);
	GpioPinConfig.GPIO_PinNumber= Pin_No_15;
	GPIO_InitPin(GpioPinConfig);

	while(1){
		uint8_t i,j=12;
		Delay();
		for(i=0 ;i<4 ;i++){
			GPIO_ToggleOutputPin(PortD, j);
			Delay();
			j++;
		}

		for(i=0 ;i<4 ;i++){
			Delay();
			//GPIO_ODRSetReset(PortD, RESET);
			for(j=12; j<=15; j++){
				k = GPIO_WriteOutputPin(PortD, j, GPIO_PIN_RESET);
				k++;
			}
			Delay();
			for(j=12; j<=15; j++)
				GPIO_ToggleOutputPin(PortD, j);
		}


	}
}

/**
 * @file main.c
 * @author Mohamed Ali Haoufa
 * @brief 
 * @version 0.1
 * @date 2023-09-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdint.h>
#include <stdio.h>
#include "../MCAL/GPIO/GPIO.h"

int main(void)
{
	// if array of pointers are static , you need to include GPIO.c
	// and if you remove static, include GPIO.h only, and extern them from GPIO.h
	// RCC_Port_AHB1ClockEnable_Registers[PortA]->Register=0x1;
	RCC_Port_ClockConfig_Register[0]->bits.MCO1= 0x01;
	RCC_Port_ClockConfig_Register[0]->bits.MCO1= 0x00;

	// access the pointer register
	volatile RCC_CR_Register *RccControlRegister = (volatile RCC_CR_Register*) RCC_ClockControl;
	RccControlRegister->bits.HSION= 1;

	volatile RCC_AHB1ENR_Register *portDClockEnableRegister = RCC_Port_AHB1ClockEnable_Registers[PortD];
	volatile GPIO_MODER_Register *portDModeSelectRegister = GPIO_ModeSelect_Registers[PortD];
	volatile GPIO_ODR_Register *portDODRControlRegister = GPIO_ODRControl_Registers[PortD];

	portDClockEnableRegister->bits.GPIODEN= 0x1;

	portDModeSelectRegister->bits.MODER12 = 0x0;
	portDModeSelectRegister->bits.MODER12 = 0x1;

	portDModeSelectRegister->bits.MODER13= 0x0;
	portDModeSelectRegister->bits.MODER13= 0x1;

	portDModeSelectRegister->bits.MODER14= 0x0;
	portDModeSelectRegister->bits.MODER14= 0x1;

	portDModeSelectRegister->bits.MODER15= 0x0;
	portDModeSelectRegister->bits.MODER15= 0x1;

	while(1){
		int i;
		for(i=0;i<2;i++){
		portDODRControlRegister->bits.ODR12^= 0x1;
		delay();
		portDODRControlRegister->bits.ODR13^= 0x1;
		delay();
		portDODRControlRegister->bits.ODR14^= 0x1;
		delay();
		portDODRControlRegister->bits.ODR15^= 0x1;
		delay();
		}

		for(i=0;i<4;i++){
		portDODRControlRegister->Register= 0x0;
		delay();
		GPIO_ToggleOutputPin(PortD, 12);
		GPIO_ToggleOutputPin(PortD, 13);
		GPIO_ToggleOutputPin(PortD, 14);
		GPIO_ToggleOutputPin(PortD, 15);
		delay();
		/*
		delay();
		portDODRControlRegister->bits.ODR12= 0x0;
		portDODRControlRegister->bits.ODR13= 0x0;
		portDODRControlRegister->bits.ODR14= 0x0;
		portDODRControlRegister->bits.ODR15= 0x0;
		delay();
		portDODRControlRegister->bits.ODR12= 0x1;
		portDODRControlRegister->bits.ODR13= 0x1;
		portDODRControlRegister->bits.ODR14= 0x1;
		portDODRControlRegister->bits.ODR15= 0x1;*/
		}

	}
}

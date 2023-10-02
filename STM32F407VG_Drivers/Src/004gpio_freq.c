/*
 * 004gpio_freq.c
 *
 *  Created on: Oct 2, 2023
 *      Author: mohamed
 */

#include "../drivers/Inc/stm32f407xx.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW


int main(void)
{

	GPIO_Handle_t GpioLed;

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinPinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeripheralClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);



	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);

	}
	return 0;
}

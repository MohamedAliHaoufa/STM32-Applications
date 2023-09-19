/**
 * 
 * @file 002led_Button.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../drivers/Inc/stm32f407xx.h"
#define HIGH         1
#define Btn_pressed  HIGH

static void delay (void)
{

    for( uint32_t i = 0 ; i < 500000/2 ; i++);

}
//we write the application here
int main(void)
{
    GPIO_Handle_t Gpioled,GPIOBtn;

    //this is led gpio configuration

    Gpioled.pGPIOx=GPIOD; //GPIO base address

    Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    Gpioled.GPIO_PinConfig.GPIO_PinPinOPType = GPIO_OP_TYPE_PP;
    Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOD,ENABLE);
    GPIO_Init(&Gpioled);

    //this is Button gpio configuration

    GPIOBtn.pGPIOx=GPIOA; //GPIO base address

    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //there is a pull down available in the shematic of the Button

    GPIO_PeripheralClockControl(GPIOA,ENABLE);
    GPIO_Init(&GPIOBtn);

    while (1)
    {
        if ( GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)== Btn_pressed)
        {
            //debouncing issues result to disturbing the toggeling and may not work properly
           
           delay();// until the debouncing of the button gets over to avoid that ,if not it'll excute toggling multiple times
           GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12) ;

        }

      }
        return 0;
    }

		

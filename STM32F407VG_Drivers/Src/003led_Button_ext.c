/**
 * @file 003led_Button_ext.c
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
#define LOW          0 
#define Btn_pressed  LOW

static void delay (void)
{

    for( uint32_t i = 0 ; i < 500000/2 ; i++);

}
//we write the application here
int main(void)
{
    GPIO_Handle_t Gpioled,GPIOBtn;

    //this is led gpio configuration

    Gpioled.pGPIOx=GPIOA; //GPIO base address

    Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8; // wa can't use the debugging pins (p269 in RF_M) as GPIO
    Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    Gpioled.GPIO_PinConfig.GPIO_PinPinOPType = GPIO_OP_TYPE_PP;
    Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; 

    GPIO_PeripheralClockControl(GPIOA,ENABLE);
    GPIO_Init(&Gpioled);

    //this is Button gpio configuration

    GPIOBtn.pGPIOx=GPIOB; //GPIO base address

    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;       //an external Button
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //we can use PU if we want , or put a external pull up 22k

    GPIO_PeripheralClockControl(GPIOB,ENABLE);
    GPIO_Init(&GPIOBtn);

    while (1)
    {
        if ( GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12)== Btn_pressed)
        {
            //debouncing issues result to disturbing the toggeling and may not work properly
           
           delay();// until the debouncing of the button gets over to avoid that ,if not it'll excute toggling multiple times
           GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8) ;

        }

      }
        return 0;
    }


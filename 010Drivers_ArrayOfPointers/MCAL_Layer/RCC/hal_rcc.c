/*
 * CAN.c
 *
 *  Created on: Sep 19, 2023
 *      Author: mohamed
 */
#include "../../MCAL_Layer/RCC/hal_rcc.h"

volatile RCC_RegDef_t static *const pRCC_RegisterControl[1] = {
			RCC_RegisterControl
};

/*
Std_ReturnType GPIO_DeInit (uint8_t PinNum){
	pRCC_RegisterControl[0]->AHB1RSTR |= (1 << PinNum) ;
	pRCC_RegisterControl[0]->AHB1RSTR &= ~(1 << PinNum) ;
    Std_ReturnType ret = E_OK;
    return ret;
}
*/

Std_ReturnType GPIO_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType I2C_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType SPI_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType UART_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType USART_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType DMA_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType TIM_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType RTC_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType CAN_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType Ethernet_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType ADC_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType DAC_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

Std_ReturnType I2S_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->AHB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}


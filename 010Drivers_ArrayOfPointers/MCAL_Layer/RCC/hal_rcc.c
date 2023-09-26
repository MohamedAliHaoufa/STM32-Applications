/**
 * @file hal_rcc.c
 * @brief This file contains the implementation of peripheral clock control functions for various peripherals.
 *
 * This source file provides the implementation of peripheral clock control functions for
 * various peripherals, such as GPIO, I2C, SPI, UART, USART, DMA, TIM, RTC, CAN, Ethernet,
 * ADC, DAC, and I2S. These functions allow enabling or disabling the clock for the specified peripheral.
 *
 * @author Mohamed
 * @date September 19, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../../MCAL_Layer/RCC/hal_rcc.h"

/**
 * @var pRCC_RegisterControl
 * @brief Array of pointers to hardware registers.
 *
 * This array contains pointers to the RCC (Reset and Clock Control) hardware registers. It is used
 * to control the peripheral clock for various peripherals by enabling or disabling the clock signals.
 */
volatile RCC_RegDef_t static *const pRCC_RegisterControl[1] = {
    RCC_RegisterControl
};

volatile uint32_t static *const pNVIC_ISERx_RegisterControl[9] = {
	NVIC_ISER0,
	NVIC_ISER1,
	NVIC_ISER2,
	NVIC_ISER3,
	NVIC_ISER4,
	NVIC_ISER5,
	NVIC_ISER6,
	NVIC_ISER7
};



Std_ReturnType NVIC_IRQEnable (uint8_t ISER_Index, uint32_t IRQNumber, uint8_t EnorDi) {

	Std_ReturnType ret = E_OK;
    if (pNVIC_ISERx_RegisterControl[ISER_Index] != NULL) {
    	switch (EnorDi) {
            case ENABLE:
            	*pNVIC_ISERx_RegisterControl[ISER_Index] |= (1<<IRQNumber);
                break;
            case DISABLE:
            	*pNVIC_ISERx_RegisterControl[ISER_Index] &= (1<<IRQNumber);
                break;
            default:
                ret = E_NOT_OK;
                break;
        }
    } else {

    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType Syscfg_ClockEnable (uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;
    if (pRCC_RegisterControl[0] != NULL) {
        switch (EnorDi) {
            case ENABLE:
            	pRCC_RegisterControl[0]->APB2ENR |= (1 << RCC_APB2ENR_SYSCFGEN);
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB2ENR &= ~(1 << RCC_APB2ENR_SYSCFGEN);
                break;
            default:
                ret = E_NOT_OK;
                break;
        }
    } else {

    	ret = E_NOT_OK;
    }
    return ret;
}
/****************************************************************************************************************
 * Deinitialize a specific peripheral
 */

/**
 * @brief Deinitialize a specific peripheral.
 *
 * This function deinitializes a specific GPIO peripheral by asserting and de-asserting the
 * peripheral's reset bit in the RCC. It is used to reset and disable a peripheral.
 *
 * @param PortNum The port number associated with the peripheral.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */

 Std_ReturnType GPIO_DeInit(uint8_t PortNum) {


    Std_ReturnType ret = E_OK;
    if (PortNum < Port_Indices) {

        pRCC_RegisterControl[0]->AHB1RSTR |= (1 << PortNum);
        pRCC_RegisterControl[0]->AHB1RSTR &= ~(1 << PortNum);

    } else {

    	ret = E_NOT_OK;
    }
    return ret;
}

/*
Std_ReturnType I2C_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if (PortNum < I2c_Indices) {

            pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
            pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {

    	ret = E_NOT_OK;
    }
    return ret;
}
*/


Std_ReturnType SPI_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if (PortNum==SPI_1) {

        pRCC_RegisterControl[0]->APB2RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB2RSTR &= ~(1 << PinNum);

    } else if ((PortNum==SPI_2) || (PortNum==SPI_3)){

    	pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType UART_DeInit (uint8_t PortNum, uint8_t PinNum){

	Std_ReturnType ret = E_OK;
    if ((PortNum==UART_4) || (PortNum==UART_5) || (PortNum==UART_7) || (PortNum==UART_8)){

    	pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType USART_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum==USART_1) || (PortNum==USART_6) ) {

        pRCC_RegisterControl[0]->APB2RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB2RSTR &= ~(1 << PinNum);

    } else if ( (PortNum >= USART_2 && PortNum <= USART_5) ){

    	pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType DMA_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum==DMA_1) || (PortNum==DMA_2) ) {

        pRCC_RegisterControl[0]->AHB1RSTR |= (1 << PortNum);
        pRCC_RegisterControl[0]->AHB1RSTR &= ~(1 << PortNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType TIM_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum==TIM_1)||(PortNum >= TIM_8 && PortNum <= TIM_11) ) {

        pRCC_RegisterControl[0]->APB2RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB2RSTR &= ~(1 << PinNum);

    } else if ((PortNum >= TIM_2 && PortNum <= TIM_7)||(PortNum >= TIM_12 && PortNum <= TIM_14)){

    	pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

// Note !!! The RTC usually has its own control for reset and is not controlled through this
// general RCC_APB1RSTR register.
// Refer to the reference manual or datasheet for your specific STM32 microcontroller
// to find the dedicated bit for resetting the RTC.
// It's often in a different control register than the one you've shown here.
Std_ReturnType RTC_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if (pRCC_RegisterControl[0] != NULL) {

            pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
            pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {

    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType CAN_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum==CAN_1) || (PortNum==CAN_2) ) {

    	pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType Ethernet_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum >= Ethernet_MAC && PortNum <= Ethernet_DMA) ){

    	pRCC_RegisterControl[0]->AHB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->AHB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType ADC_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum >= ADC_1 && PortNum <= ADC_3) ){

    	pRCC_RegisterControl[0]->APB2RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB2RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType DAC_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum >= DAC_1 && PortNum <= DAC_2) ){

    	pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

Std_ReturnType I2S_DeInit (uint8_t PortNum, uint8_t PinNum){

    Std_ReturnType ret = E_OK;
    if ( (PortNum >= I2Sext_2 && PortNum <= I2Sext_3) ){

    	pRCC_RegisterControl[0]->APB1RSTR |= (1 << PinNum);
        pRCC_RegisterControl[0]->APB1RSTR &= ~(1 << PinNum);

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/****************************************************************************************************************
 * Control a specific peripheral clock
 */

/**
 * @brief Control the peripheral clock for GPIO.
 *
 * This function enables or disables the peripheral clock for GPIO by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the GPIO peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_PeripheralClockControl(uint8_t PortNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;
    switch (EnorDi) {
        case ENABLE:
            pRCC_RegisterControl[0]->AHB1ENR |= (1 << PortNum);
            break;
        case DISABLE:
            pRCC_RegisterControl[0]->AHB1ENR &= ~(1 << PortNum);
            break;
        default:
            ret = E_NOT_OK;
            break;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for I2C.
 *
 * This function enables or disables the peripheral clock for I2C by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the I2C peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType I2C_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;
    if (PortNum < I2c_Indices) {
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {

    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for SPI.
 *
 * This function enables or disables the peripheral clock for SPI by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the SPI peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType SPI_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;
    if (PortNum==SPI_1) {
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB2ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB2ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else if ((PortNum==SPI_2) || (PortNum==SPI_3)){
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for UART.
 *
 * This function enables or disables the peripheral clock for UART by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the UART peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType UART_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ((PortNum==UART_4) || (PortNum==UART_5) || (PortNum==UART_7) || (PortNum==UART_8)){
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for USART.
 *
 * This function enables or disables the peripheral clock for USART by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the USART peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType USART_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ( (PortNum==USART_1) || (PortNum==USART_6) ) {
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB2ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB2ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else if ( (PortNum >= USART_2 && PortNum <= USART_5) ){
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for DMA.
 *
 * This function enables or disables the peripheral clock for DMA by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the DMA peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType DMA_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ( (PortNum==DMA_1) || (PortNum==DMA_2) ) {
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

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for TIM.
 *
 * This function enables or disables the peripheral clock for TIM by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the TIM peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType TIM_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;
    if ( (PortNum==TIM_1)||(PortNum >= TIM_8 && PortNum <= TIM_11) ) {
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB2ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB2ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else if ((PortNum >= TIM_2 && PortNum <= TIM_7)||(PortNum >= TIM_12 && PortNum <= TIM_14)){
    	switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }

    return ret;
}

/**
 * @brief Control the peripheral clock for RTC.
 *
 * This function enables or disables the peripheral clock for RTC by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the RTC peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType RTC_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
        	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
            break;
        case DISABLE:
        	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for CAN.
 *
 * This function enables or disables the peripheral clock for CAN by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the CAN peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType CAN_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ( (PortNum==CAN_1) || (PortNum==CAN_2) ) {

        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for Ethernet.
 *
 * This function enables or disables the peripheral clock for Ethernet by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the Ethernet peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType Ethernet_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ( (PortNum >= Ethernet_MAC && PortNum <= Ethernet_DMA) ){

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

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for ADC.
 *
 * This function enables or disables the peripheral clock for ADC by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the ADC peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType ADC_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ( (PortNum >= ADC_1 && PortNum <= ADC_3) ){
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB2ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB2ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for DAC.
 *
 * This function enables or disables the peripheral clock for DAC by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the DAC peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType DAC_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ( (PortNum >= DAC_1 && PortNum <= DAC_2) ){
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Control the peripheral clock for I2S.
 *
 * This function enables or disables the peripheral clock for I2S by setting or clearing the
 * corresponding bit in the RCC's AHB1ENR register.
 *
 * @param PortNum The port number associated with the peripheral.
 * @param PinNum The pin number associated with the I2S peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable it.
 * @return Std_ReturnType E_OK if the operation is successful, E_NOT_OK otherwise.
 */
Std_ReturnType I2S_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi) {

    Std_ReturnType ret = E_OK;

    if ( (PortNum >= I2Sext_2 && PortNum <= I2Sext_3) ){
        switch(EnorDi){
            case ENABLE:
            	pRCC_RegisterControl[0]->APB1ENR |= (1 << PinNum) ;
                break;
            case DISABLE:
            	pRCC_RegisterControl[0]->APB1ENR &= ~(1 << PinNum) ;
                break;
            default: ret = E_NOT_OK;
                break;
        }

    } else {
    	ret = E_NOT_OK;
    }
    return ret;
}

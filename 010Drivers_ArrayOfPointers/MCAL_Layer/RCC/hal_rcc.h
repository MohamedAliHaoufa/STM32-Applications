/*
 * hal_rcc.h
 *
 *  Created on: Sep 22, 2023
 *      Author: mohamed
 */

#ifndef HAL_RCC_H_
#define HAL_RCC_H_
#include "hal_rcc_reg.h"
#include "hal_rcc_cfg.h"

//Std_ReturnType GPIO_DeInit (uint8_t PinNum);

Std_ReturnType GPIO_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType I2C_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType SPI_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType UART_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType USART_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType DMA_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType TIM_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType RTC_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType CAN_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType Ethernet_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType ADC_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType DAC_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType I2S_PeripheralClockControl (uint8_t PinNum, uint8_t EnorDi);

#endif /* HAL_RCC_H_ */

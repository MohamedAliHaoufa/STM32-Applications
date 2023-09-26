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

Std_ReturnType NVIC_IRQEnable (uint8_t channel, uint32_t IRQNumber, uint8_t EnorDi);
Std_ReturnType Syscfg_ClockEnable (uint8_t EnorDi);
Std_ReturnType GPIO_DeInit (uint8_t PortNum);
//Std_ReturnType I2C_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType SPI_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType UART_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType USART_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType DMA_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType TIM_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType RTC_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType CAN_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType Ethernet_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType ADC_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType DAC_DeInit (uint8_t PortNum, uint8_t PinNum);
Std_ReturnType I2S_DeInit (uint8_t PortNum, uint8_t PinNum);

Std_ReturnType GPIO_PeripheralClockControl (uint8_t PortNum, uint8_t EnorDi);
Std_ReturnType I2C_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType SPI_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType UART_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType USART_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType DMA_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType TIM_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType RTC_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType CAN_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType Ethernet_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType ADC_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType DAC_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);
Std_ReturnType I2S_PeripheralClockControl(uint8_t PortNum, uint8_t PinNum, uint8_t EnorDi);

#endif /* HAL_RCC_H_ */

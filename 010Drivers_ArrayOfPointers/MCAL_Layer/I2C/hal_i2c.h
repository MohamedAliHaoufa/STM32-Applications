/**
 * @file stm32f407xx_i2c_driver.h
 * @brief This file contains definitions and functions prototypes for the STM32F407xx I2C driver.
 * @version 0.1
 * @date 2023-08-10
 *
 * @note Copyright (c) 2023
 *
 */

#ifndef HAL_I2C_H_
#define HAL_I2C_H_

#include "hal_i2c_cfg.h"
#include "hal_i2c_reg.h"


/**
 * @defgroup I2C_APIS I2C APIs
 * @brief APIs supported by the I2C driver.
 * @{
 */

/**
 * @brief Controls the peripheral clock of the I2C peripheral.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 * @param EnorDi ENABLE to enable the clock, DISABLE to disable.
 */
void I2C_PeriClockControl(I2C_Handle_t *pI2CHandle, uint8_t EnorDi );

/**
 * @brief Initializes the I2C peripheral.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_Init( I2C_Handle_t *pI2CHandle );

/**
 * @brief Deinitializes the I2C peripheral.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 */
void I2C_DeInit( I2C_Handle_t *pI2CHandle );

/**
 * @brief Retrieves the PCLK1 value used for I2C configuration.
 *
 * @return uint32_t PCLK1 value in Hz.
 */
uint32_t RCC_GetPCLK1Value(void);

/**
 * @brief Retrieves the PLL output clock used for I2C configuration.
 *
 * @return uint32_t PLL output clock in Hz.
 */
uint32_t RCC_GETPLLOutputClock(void);

/**
 * @brief Sends data in master mode over the I2C bus.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pTxBuffer Pointer to the transmit buffer.
 * @param Len Length of data to be sent.
 * @param SlaveAddr Slave address to communicate with.
 * @param SR Repeated start setting.
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t SR);

/**
 * @brief Receives data in master mode over the I2C bus.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pRxBuffer Pointer to the receive buffer.
 * @param Len Length of data to be received.
 * @param SlaveAddr Slave address to communicate with.
 * @param SR Repeated start setting.
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t SR);

/**
 * @brief Sends a single byte of data in slave mode over the I2C bus.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 * @param data Data to be sent.
 */
void I2C_SlaveSendData( I2C_Handle_t *pI2CHandle , uint8_t data );

/**
 * @brief Receives a single byte of data in slave mode over the I2C bus.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 * @return uint8_t Received data.
 */
uint8_t I2C_SlaveReceiveData( I2C_Handle_t *pI2CHandle );

/**
 * @brief Sends data in master mode over the I2C bus with interrupt support.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pTxBuffer Pointer to the transmit buffer.
 * @param Len Length of data to be sent.
 * @param SlaveAddr Slave address to communicate with.
 * @param SR Repeated start setting.
 * @return uint8_t Status flag.
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t SR);

/**
 * @brief Receives data in master mode over the I2C bus with interrupt support.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pRxBuffer Pointer to the receive buffer.
 * @param Len Length of data to be received.
 * @param SlaveAddr Slave address to communicate with.
 * @param SR Repeated start setting.
 * @return uint8_t Status flag.
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t SR);

/**
 * @brief Closes the transmission process.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/**
 * @brief Closes the reception process.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/**
 * @brief Configures IRQ interrupt for the I2C peripheral.
 *
 * @param IRQNumber IRQ number.
 * @param EnorDi ENABLE to enable the IRQ, DISABLE to disable.
 */
void I2C_IRQInterruptConfig( uint8_t IRQNumber, uint8_t EnorDi );

/**
 * @brief Configures IRQ priority for the I2C peripheral.
 *
 * @param IRQNumber IRQ number.
 * @param IRQPriority IRQ priority value.
 */
void I2C_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority );

/**
 * @brief Handles I2C event interrupt.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_EV_IRQHandling( I2C_Handle_t *pI2CHandle);

/**
 * @brief Handles I2C error interrupt.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_ER_IRQHandling( I2C_Handle_t *pI2CHandle);

/**
 * @brief Controls peripheral operation in master mode.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 * @param EnorDi ENABLE to enable, DISABLE to disable.
 */
void I2C_PeripheralControl( I2C_Handle_t *pI2CHandle , uint8_t EnorDi );

/**
 * @brief Retrieves the flag status of the specified I2C flag.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 * @param FlagName Flag to check.
 * @return uint8_t FLAG_SET if flag is set, FLAG_RESET if not.
 */
uint8_t I2C_GetFlagStatus( I2C_Handle_t *pI2CHandle, uint8_t FlagName );

/**
 * @brief Manages ACKing during I2C communication.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 * @param EnorDi I2C_ACK_ENABLE to enable ACKing, I2C_ACK_DISABLE to disable.
 */
void I2C_ManageAcking( I2C_Handle_t *pI2CHandle, uint8_t EnorDi );

/**
 * @brief Generates a stop condition on the I2C bus.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 */
void I2C_GenerateStopCondition(I2C_Handle_t *pI2CHandle);

/**
 * @brief Enables or disables callback events for the I2C slave.
 *
 * @param pI2Cx Pointer to the I2C peripheral.
 * @param EnorDi ENABLE to enable callback events, DISABLE to disable.
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_Handle_t *pI2CHandle, uint8_t EnorDi);

/**
 * @brief Handles application events for the I2C communication.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param AppEv Application event to be handled.
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

/** @} */



#endif /* HAL_I2C_H_ */

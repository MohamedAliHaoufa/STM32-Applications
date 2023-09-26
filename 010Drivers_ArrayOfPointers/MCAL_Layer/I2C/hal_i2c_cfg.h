/*
 * hal_i2c_cfg.h
 *
 *  Created on: Sep 19, 2023
 *      Author: mohamed
 */

#ifndef HAL_I2C_CFG_H_
#define HAL_I2C_CFG_H_

#include "../mcal_stm32f407xx.h"

typedef struct
{
    uint32_t I2C_SCLspeed;
    uint8_t I2C_DeviceAdress;   // slave address is 7 bits wide
    uint8_t I2C_ACKControl;
    uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct
{
    I2C_Config_t I2C_Config;
    uint8_t I2cIndex;
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxRxState;
    uint8_t DevAddr;
    uint32_t RxSize;
    uint8_t Sr;
} I2C_Handle_t;

/**
 * @defgroup I2C_Macros I2C Macros
 * @brief Macros related to I2C configuration, flags, and events.
 * @{
 */

/**
 * @defgroup I2C_Application_States I2C Application States
 * @brief Possible states of the I2C application.
 * @{
 */
#define I2C_READY           0 /**< I2C application is ready for communication. */
#define I2C_BUSY_IN_RX      1 /**< I2C application is busy receiving data. */
#define I2C_BUSY_IN_TX      2 /**< I2C application is busy transmitting data. */
/** @} */

/**
 * @defgroup I2C_SCL_Speed_Options I2C Serial Clock Speed Options
 * @brief Options for I2C serial clock speed.
 * @{
 */
#define I2C_SC_SPEED_SM     100000   /**< Standard mode serial clock speed, up to 100 KHz. */
#define I2C_SC_SPEED_FM4K   400000   /**< Fast mode serial clock speed, up to 400 KHz. */
#define I2C_SC_SPEED_FM2K   200000   /**< Fast mode serial clock speed, up to 200 KHz. */
/** @} */

/**
 * @defgroup I2C_ACK_Control_Options I2C ACK Control Options
 * @brief Options for controlling the ACK mechanism in I2C communication.
 * @{
 */
#define I2C_ACK_ENABLE      1 /**< Enable ACK mechanism. */
#define I2C_ACK_DISABLE     0 /**< Disable ACK mechanism (default behavior). */
/** @} */

/**
 * @defgroup I2C_FM_Duty_Cycle_Options I2C Fast Mode Duty Cycle Options
 * @brief Options for fast mode duty cycle in I2C communication.
 * @{
 */
#define I2C_FM_DUTY_2       0 /**< Fast mode duty cycle option: Tlow/THigh = 2. */
#define I2C_FM_DUTY_16_9    1 /**< Fast mode duty cycle option: Tlow/THigh = 16/9. */
/** @} */

/**
 * @defgroup I2C_Flags I2C Status Flags
 * @brief Flags indicating various status conditions in I2C communication.
 * @{
 */
#define I2C_FLAG_SB            (1 << I2C_SR1_SB)      /**< Start bit flag. */
#define I2C_FLAG_ADDR          (1 << I2C_SR1_ADDR)    /**< Address sent flag. */
#define I2C_FLAG_BTF           (1 << I2C_SR1_BTF)     /**< Byte transfer finished flag. */
#define I2C_FLAG_ADD10         (1 << I2C_SR1_ADD10)   /**< 10-bit header sent flag. */
#define I2C_FLAG_STOPF         (1 << I2C_SR1_STOPF)   /**< Stop detection flag. */
#define I2C_FLAG_RxNE          (1 << I2C_SR1_RxNE)    /**< Receive data register not empty flag. */
#define I2C_FLAG_TxE           (1 << I2C_SR1_TxE)     /**< Transmit data register empty flag. */
#define I2C_FLAG_BERR          (1 << I2C_SR1_BERR)    /**< Bus error flag. */
#define I2C_FLAG_ARLO          (1 << I2C_SR1_ARLO)    /**< Arbitration lost error flag. */
#define I2C_FLAG_AF            (1 << I2C_SR1_AF)      /**< Acknowledge failure flag. */
#define I2C_FLAG_OVR           (1 << I2C_SR1_OVR)     /**< Overrun/underrun error flag. */
#define I2C_FLAG_PECERR        (1 << I2C_SR1_PECERR)  /**< PEC error in reception flag. */
#define I2C_FLAG_TIMEOUT       (1 << I2C_SR1_TIMEOUT) /**< Timeout error flag. */
#define I2C_FLAG_SMBALERT      (1 << I2C_SR1_SMBALERT)/**< SMBus alert flag. */
/** @} */

/**
 * @defgroup I2C_Repeated_Start_Macros I2C Repeated Start Macros
 * @brief Macros for enabling or disabling repeated start conditions in I2C communication.
 * @{
 */
#define I2C_ENABLE_Sr      SET    /**< Macro to enable repeated start condition (SET). */
#define I2C_DISABLE_Sr     RESET  /**< Macro to disable repeated start condition (RESET). */
/** @} */

/**
 * @defgroup I2C_Application_Event_Macros I2C Application Event Macros
 * @brief Macros for I2C application events.
 * @{
 */
#define I2C_EV_TX_CMPLT     0 /**< Event: Transmission complete. */
#define I2C_EV_RX_CMPLT     1 /**< Event: Reception complete. */
#define I2C_EV_STOP         2 /**< Event: Stop condition. */
#define I2C_ERROR_BERR      3 /**< Event: Bus error. */
#define I2C_ERROR_ARLO      4 /**< Event: Arbitration lost error. */
#define I2C_ERROR_AF        5 /**< Event: Acknowledge failure. */
#define I2C_ERROR_OVR       6 /**< Event: Overrun/underrun error. */
#define I2C_ERROR_TIMEOUT   7 /**< Event: Timeout error. */
#define I2C_EV_DATA_REQ     8 /**< Event: Data request. */
#define I2C_EV_DATA_RCV     9 /**< Event: Data reception. */
/** @} */

/** @} */ // End of I2C_Macros group

#endif /* HAL_I2C_CFG_H_ */

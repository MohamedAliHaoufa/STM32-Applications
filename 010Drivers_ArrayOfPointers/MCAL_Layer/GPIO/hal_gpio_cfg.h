/**
 * @file hal_gpio_cfg.h
 * @brief This file contains configurations and definitions for GPIO (General Purpose Input/Output) functionality.
 *
 * This header file defines various configurations and structures related to GPIO pin configuration
 * and control. It provides macros, enums, and data structures to simplify the configuration
 * of GPIO pins on a microcontroller.
 *
 * @author Mohamed
 * @date September 16, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef HAL_GPIO_CFG_H_
#define HAL_GPIO_CFG_H_

#include "../mcal_stm32f407xx.h"

/**
 * @struct GPIO_PinConfig_t
 * @brief GPIO pin configuration structure.
 *
 * This structure holds the configuration settings for a GPIO pin, including its mode, speed,
 * pull-up/pull-down configuration, output type, and alternate function mode.
 */
typedef struct
{
    uint32_t GPIO_PortIndex;        /*!< Specifies the GPIO port index. */
    uint32_t GPIO_PinNumber;        /*!< Specifies the GPIO pin number. */
    uint32_t GPIO_PinMode;          /*!< Specifies the mode of the GPIO pin. */
    uint32_t GPIO_PinSpeed;         /*!< Specifies the speed of the GPIO pin. */
    uint32_t GPIO_PinPuPdControl;   /*!< Specifies the pull-up/pull-down configuration for the GPIO pin. */
    uint32_t GPIO_PinPinOPType;     /*!< Specifies the output type of the GPIO pin. */
    uint32_t GPIO_PinAltFunMode;    /*!< Specifies the alternate function mode of the GPIO pin. */
} GPIO_PinConfig_t; /*!< Holds GPIO pin configuration settings. */

/**
 * @enum GPIO_ConfigurePinNum_t
 * @brief Possible GPIO pin numbers.
 */
typedef enum {
    Pin_No_0 = 0,
    Pin_No_1 = 1,
    Pin_No_2 = 2,
    Pin_No_3 = 3,
    Pin_No_4 = 4,
    Pin_No_5 = 5,
    Pin_No_6 = 6,
    Pin_No_7 = 7,
    Pin_No_8 = 8,
    Pin_No_9 = 9,
    Pin_No_10 = 10,
    Pin_No_11 = 11,
    Pin_No_12 = 12,
    Pin_No_13 = 13,
    Pin_No_14 = 14,
    Pin_No_15 = 15,

    Pin_Indices = 16,
} GPIO_ConfigurePinNum_t;

/**
 * @enum GPIO_PortNumIndexArr_t
 * @brief Possible GPIO port indices.
 */
typedef enum {
    PortA = 0,
    PortB = 1,
    PortC = 2,
    PortD = 3,
    PortE = 4,
    PortF = 5,
    PortG = 6,
    PortH = 7,
    PortI = 8,

    Port_Indices = 9
} GPIO_PortNumIndexArr_t;

typedef enum {
    SPI_1 = 0,
	SPI_2 = 1,
	SPI_3 = 2,
	SPI_4 = 3,
	SPI_5 = 4,
	SPI_6 = 5,

    SPI_Indices = 6
} SPIIndexArr_t;

typedef enum {
    I2C_1 = 0,
	I2C_2 = 1,
	I2C_3 = 2,

    I2c_Indices = 3
} I2CIndexArr_t;

typedef enum {
    UART_1 = 0,
	UART_2 = 1,
	UART_3 = 2,
	UART_4 = 3,
	UART_5 = 4,
	UART_6 = 5,
	UART_7 = 6,
	UART_8 = 7,

    UART_Indices = 8
} UARTIndexArr_t;

typedef enum {
    USART_1 = 0,
	USART_2 = 1,
	USART_3 = 2,
	USART_4 = 3,
	USART_5 = 4,
	USART_6 = 5,

    USART_Indices = 6
} USARTIndexArr_t;

typedef enum {
    DMA_1 = 0,
    DMA_2 = 1,

    DMA_Indices = 2
} DMAIndexArr_t;

typedef enum {
    TIM_1 = 0,
	TIM_2 = 1,
	TIM_3 = 2,
	TIM_4 = 3,
	TIM_5 = 4,
	TIM_6 = 5,
	TIM_7 = 6,
	TIM_8 = 7,
	TIM_9 = 8,
	TIM_10 = 9,
	TIM_11 = 10,
	TIM_12 = 11,
	TIM_13 = 12,
	TIM_14 = 13,

    TIM_Indices = 14
} TIMIndexArr_t;

typedef enum {
    CAN_1 = 0,
    CAN_2 = 1,

    CAN_Indices = 2
} CANIndexArr_t;

typedef enum {
    Ethernet_MAC = 0,
	Ethernet_MMC = 1,
	Ethernet_PTP = 2,
	Ethernet_DMA = 3,

    Ethernet_Indices = 4
} EthernetIndexArr_t;

typedef enum {
    OTG_FS_GLOBAL = 0,
	OTG_FS_HOST   = 1,
	OTG_FS_DEVICE = 2,
	OTG_FS_PWRCLK = 3,

    OTG_HS_GLOBAL = 4,
	OTG_HS_HOST   = 5,
	OTG_HS_DEVICE = 6,
	OTG_HS_PWRCLK = 7,

    OTG_Indices = 8
} OTGIndexArr_t;

typedef enum {
    ADC_1 = 0,
    ADC_2 = 1,
    ADC_3 = 2,

    ADC_Indices = 3
} ADCIndexArr_t;

typedef enum {
    DAC_1 = 0,
    DAC_2 = 1,

    DAC_Indices = 2
} DACIndexArr_t;

typedef enum {
    I2S_2 = 0,
    I2S_3 = 1,

    I2S_Indices = 2
} I2SIndexArr_t;

typedef enum {
    I2Sext_2 = 0,
    I2Sext_3 = 1,

    I2Sext_Indices = 2
} I2SextIndexArr_t;
/**
 * @defgroup GPIO_MACROS GPIO Macros
 * @brief Macros for GPIO configuration and settings.
 * @{
 */

/**
 * @defgroup GPIO_PINBERS GPIO Pin Numbers
 * @brief Possible GPIO pin numbers.
 * @{
 */
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10   10
#define GPIO_PIN_NO_11   11
#define GPIO_PIN_NO_12   12
#define GPIO_PIN_NO_13   13
#define GPIO_PIN_NO_14   14
#define GPIO_PIN_NO_15   15
/** @} */

/**
 * @defgroup GPIO_PIN_MODES GPIO Pin Modes
 * @brief Possible GPIO pin modes.
 * @{
 */
#define GPIO_MODE_IN          0 /*!< GPIO Input mode. */
#define GPIO_MODE_OUT         1 /*!< GPIO Output mode. */
#define GPIO_MODE_ALTFN       2 /*!< GPIO Alternate Function mode. */
#define GPIO_MODE_ANALOG      3 /*!< GPIO Analog mode. */
#define GPIO_MODE_IT_FT       4 /*!< GPIO Interrupt Falling-Edge Trigger mode. */
#define GPIO_MODE_IT_RT       5 /*!< GPIO Interrupt Rising-Edge Trigger mode. */
#define GPIO_MODE_IT_RFT      6 /*!< GPIO Interrupt Rising-Falling Edge Trigger mode. */
/** @} */

/**
 * @defgroup GPIO_SPEEDS GPIO Output Speeds
 * @brief Possible GPIO pin output speeds.
 * @{
 */
#define GPIO_SPEED_LOW        0 /*!< GPIO Output speed Low. */
#define GPIO_SPEED_MEDIUM     1 /*!< GPIO Output speed Medium. */
#define GPIO_SPEED_FAST       2 /*!< GPIO Output speed Fast. */
#define GPIO_SPEED_HIGH       3 /*!< GPIO Output speed High. */
/** @} */

/**
 * @defgroup GPIO_PUPD GPIO Pull-up/Pull-down Configurations
 * @brief Possible GPIO pin pull-up and pull-down configurations.
 * @{
 */
#define GPIO_NO_PUPD          0 /*!< No pull-up/pull-down configuration. */
#define GPIO_PIN_PU           1 /*!< GPIO Pull-up configuration. */
#define GPIO_PIN_PD           2 /*!< GPIO Pull-down configuration. */
/** @} */

/**
 * @defgroup GPIO_OUTPUT_TYPES GPIO Output Types
 * @brief Possible GPIO pin output types.
 * @{
 */
#define GPIO_OP_TYPE_PP       0 /*!< GPIO Output type Push-Pull mode. */
#define GPIO_OP_TYPE_OD       1 /*!< GPIO Output type Open-Drain mode. */
/** @} */

/** @} */ // End of GPIO Macros group

#endif /* HAL_GPIO_CFG_H_ */

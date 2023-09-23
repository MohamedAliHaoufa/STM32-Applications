/*
 * hal_gpio_cfg.h
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#ifndef HAL_GPIO_CFG_H_
#define HAL_GPIO_CFG_H_

#include "../mcal_stm32f407xx.h"

typedef struct
{
    uint32_t GPIO_PortIndex;        /*!< Specifies the GPIO pin number. */
    uint32_t GPIO_PinNumber;        /*!< Specifies the GPIO pin number. */
    uint32_t GPIO_PinMode;          /*!< Specifies the mode of the GPIO pin. */
    uint32_t GPIO_PinSpeed;         /*!< Specifies the speed of the GPIO pin. */
    uint32_t GPIO_PinPuPdControl;   /*!< Specifies the pull-up/pull-down configuration for the GPIO pin. */
    uint32_t GPIO_PinPinOPType;     /*!< Specifies the output type of the GPIO pin. */
    uint32_t GPIO_PinAltFunMode;    /*!< Specifies the alternate function mode of the GPIO pin. */
} GPIO_PinConfig_t; /*!< Holds GPIO pin configuration settings. */

typedef enum{

    Pin_No_0 =0,
    Pin_No_1 =1,
    Pin_No_2 =2,
    Pin_No_3 =3,
    Pin_No_4 =4,
    Pin_No_5 =5,
    Pin_No_6 =6,
    Pin_No_7 =7,
    Pin_No_8 =8,
    Pin_No_9 =9,
    Pin_No_10 =10,
    Pin_No_11 =11,
    Pin_No_12 =12,
    Pin_No_13 =13,
    Pin_No_14 =14,
    Pin_No_15 =15,

}GPIO_ConfigurePinNum_t;

typedef enum{
    PortA =0,
    PortB =1,
    PortC =2,
    PortD =3,
    PortE =4,
    PortF =5,
    PortG =6,
    PortH =7,
    PortI =8,

    Port_Indices=9
}GPIO_PortNumIndexArr_t;

/**
 * @defgroup GPIO_MACROS GPIO Macros
 * @brief Macros for GPIO configuration and settings.
 * @{
 */

/**
 * @defgroup GPIO_PIN_NUMBERS GPIO Pin Numbers
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

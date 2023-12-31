/*
 * GPIO_reg.h
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#ifndef GPIO_REG_H_
#define GPIO_REG_H_

#include "../MCAL/std_libraries.h"
#include "../MCAL/mcal_std_types.h"

/**************************************************************************************
 **************************** Base Addresses ******************************************
 **************************************************************************************/

/**************************************************************************************
 * GPIOx Base Addresses
 */
#define PORTA_BASE  0x40020000
#define PORTB_BASE  0x40020400
#define PORTC_BASE  0x40020800
#define PORTD_BASE  0x40020C00
#define PORTE_BASE  0x40021000
#define PORTF_BASE  0x40021400

/**************************************************************************************
 * RCCx Base Addresses
 */
#define RCC_BASE    0x40023800

/**************************************************************************************
 **************************** Peripheral Register Definition **************************
 **************************************************************************************/

/**************************************************************************************
 * RCC Clock Control Register
 */
typedef union {
    struct {
        uint32_t HSION : 1;
        uint32_t HSIRDY : 1;
        uint32_t Reserved : 1;
        uint32_t HSITRIM : 5;
        uint32_t HSICAL : 8;
        uint32_t HSEON :1;
        uint32_t HSERDY : 1;
        uint32_t HSEBYP : 1;
        uint32_t CSSON : 1;
        uint32_t Reserved1 : 4;
        uint32_t PLLON : 1;
        uint32_t PLLRDY : 1;
        uint32_t PLLI2SON : 1;
        uint32_t PLLI2SRDY : 1;
        uint32_t Reserved2 : 4;
    } bits;
    uint32_t Register;
} RCC_CR_Register;


/**************************************************************************************
 * RCC clock configuration register
 */
typedef union {
    struct {
        uint32_t SW0 : 1;
        uint32_t SW1 : 1;
        uint32_t SWS0 : 1;
        uint32_t SWS1 : 1;
        uint32_t HPRE : 4;
        uint32_t Reserved :2;
        uint32_t PPRE1 : 3;
        uint32_t PPRE2 : 3;
        uint32_t RTCPRE : 5;
        uint32_t MCO1 : 2;
        uint32_t I2SSCR : 1;
        uint32_t MCO1PRE : 3;
        uint32_t MCO2PRE : 3;
        uint32_t MCO2 : 2;
    } bits;
    uint32_t Register;
} RCC_CFGR_Register;


/**************************************************************************************
 * RCC AHB1 peripheral clock register
 */
typedef union {
    struct {
        uint32_t GPIOAEN : 1;
        uint32_t GPIOBEN : 1;
        uint32_t GPIOCEN : 1;
        uint32_t GPIODEN : 1;
        uint32_t GPIOEEN : 1;
        uint32_t GPIOFEN : 1;
        uint32_t GPIOGEN : 1;
        uint32_t GPIOHEN : 1;
        uint32_t GPIOIEN : 1;
        uint32_t Reserved1 : 3;
        uint32_t CRCEN : 1;
        uint32_t Reserved2 : 5;
        uint32_t BKPSRAMEN : 1;
        uint32_t Reserved3 :1;
        uint32_t CCMDATARAMEN : 1;
        uint32_t DMA1EN : 1;
        uint32_t DMA2EN : 1;
        uint32_t Reserved4 :2;
        uint32_t ETHMACEN : 1;
        uint32_t ETHMACTXEN : 1;
        uint32_t ETHMACRXEN : 1;
        uint32_t ETHMACPTPEN : 1;
        uint32_t OTGHSEN : 1;
        uint32_t OTGHSULPIEN : 1;
        uint32_t Reserved5 :1;
    } bits;
    uint32_t Register;
} RCC_AHB1ENR_Register;

/**************************************************************************************
 * GPIO Port Mode Register
 */
typedef union {
    struct {
        uint32_t MODER0 : 2;
        uint32_t MODER1 : 2;
        uint32_t MODER2 : 2;
        uint32_t MODER3 : 2;
        uint32_t MODER4 : 2;
        uint32_t MODER5 : 2;
        uint32_t MODER6 : 2;
        uint32_t MODER7 : 2;
        uint32_t MODER8 : 2;
        uint32_t MODER9 : 2;
        uint32_t MODER10 : 2;
        uint32_t MODER11 : 2;
        uint32_t MODER12 : 2;
        uint32_t MODER13 : 2;
        uint32_t MODER14 : 2;
        uint32_t MODER15 : 2;
    } bits;
    uint32_t Register;
} GPIO_MODER_Register;

/**************************************************************************************
 * GPIO port input data register
 */
typedef union {
    struct {
        uint32_t IDR0 : 1;
        uint32_t IDR1 : 1;
        uint32_t IDR2 : 1;
        uint32_t IDR3 : 1;
        uint32_t IDR4 : 1;
        uint32_t IDR5 : 1;
        uint32_t IDR6 : 1;
        uint32_t IDR7 : 1;
        uint32_t IDR8 : 1;
        uint32_t IDR9 : 1;
        uint32_t IDR10 : 1;
        uint32_t IDR11 : 1;
        uint32_t IDR12 : 1;
        uint32_t IDR13 : 1;
        uint32_t IDR14 : 1;
        uint32_t IDR15 : 1;
        uint32_t Reserved :16;
    } bits;
    uint32_t Register;
} GPIO_IDR_Register;

/**************************************************************************************
 * GPIO port output data register
 */
typedef union {
    struct {
        uint32_t ODR0 : 1;
        uint32_t ODR1 : 1;
        uint32_t ODR2 : 1;
        uint32_t ODR3 : 1;
        uint32_t ODR4 : 1;
        uint32_t ODR5 : 1;
        uint32_t ODR6 : 1;
        uint32_t ODR7 : 1;
        uint32_t ODR8 : 1;
        uint32_t ODR9 : 1;
        uint32_t ODR10 : 1;
        uint32_t ODR11 : 1;
        uint32_t ODR12 : 1;
        uint32_t ODR13 : 1;
        uint32_t ODR14 : 1;
        uint32_t ODR15 : 1;
        uint32_t Reserved :16;
    } bits;
    uint32_t Register;
} GPIO_ODR_Register;

/**************************************************************************************
 * GPIO Alternate Function Registers
 */

typedef union {
    struct {
        uint32_t AFRL0 : 4;     // AFRL0 bits
        uint32_t AFRL1 : 4;     // AFRL1 bits
        uint32_t AFRL2 : 4;     // AFRL2 bits
        uint32_t AFRL3 : 4;     // AFRL3 bits
        uint32_t AFRL4 : 4;     // AFRL4 bits
        uint32_t AFRL5 : 4;     // AFRL5 bits
        uint32_t AFRL6 : 4;     // AFRL6 bits
        uint32_t AFRL7 : 4;     // AFRL7 bits
    } bits;
    uint32_t Register;
} GPIO_AFRL_Register;

typedef union {
    struct {
        uint32_t AFRH8 : 4;     // AFRH8 bits
        uint32_t AFRH9 : 4;     // AFRH9 bits
        uint32_t AFRH10 : 4;    // AFRH10 bits
        uint32_t AFRH11 : 4;    // AFRH11 bits
        uint32_t AFRH12 : 4;    // AFRH12 bits
        uint32_t AFRH13 : 4;    // AFRH13 bits
        uint32_t AFRH14 : 4;    // AFRH14 bits
        uint32_t AFRH15 : 4;    // AFRH15 bits
    } bits;
    uint32_t Register;
} GPIO_AFRH_Register;


/**************************************************************************************
 **************************** Macros Definition ***************************************
 **************************************************************************************/

#define RCC_ClockControl ((volatile RCC_CR_Register*)(RCC_BASE +  0x00))
#define RCC_ClockConfig  ((volatile RCC_CFGR_Register*)(RCC_BASE +  0x08))

#define RCC_PortA_AHB1ClockEnable ((volatile RCC_AHB1ENR_Register*)(RCC_BASE +  0x30))
#define RCC_PortB_AHB1ClockEnable ((volatile RCC_AHB1ENR_Register*)(RCC_BASE +  0x30))
#define RCC_PortC_AHB1ClockEnable ((volatile RCC_AHB1ENR_Register*)(RCC_BASE +  0x30))
#define RCC_PortD_AHB1ClockEnable ((volatile RCC_AHB1ENR_Register*)(RCC_BASE +  0x30))
#define RCC_PortE_AHB1ClockEnable ((volatile RCC_AHB1ENR_Register*)(RCC_BASE +  0x30))
#define RCC_PortF_AHB1ClockEnable ((volatile RCC_AHB1ENR_Register*)(RCC_BASE +  0x30))

#define GPIO_PortA_IDRControl (( volatile GPIO_IDR_Register*)(PORTA_BASE +  0x10))
#define GPIO_PortB_IDRControl (( volatile GPIO_IDR_Register*)(PORTB_BASE +  0x10))
#define GPIO_PortC_IDRControl (( volatile GPIO_IDR_Register*)(PORTC_BASE +  0x10))
#define GPIO_PortD_IDRControl (( volatile GPIO_IDR_Register*)(PORTD_BASE +  0x10))
#define GPIO_PortE_IDRControl (( volatile GPIO_IDR_Register*)(PORTE_BASE +  0x10))
#define GPIO_PortF_IDRControl (( volatile GPIO_IDR_Register*)(PORTF_BASE +  0x10))

#define GPIO_PortA_ODRControl ((volatile GPIO_ODR_Register*)(PORTA_BASE +  0x14))
#define GPIO_PortB_ODRControl ((volatile GPIO_ODR_Register*)(PORTB_BASE +  0x14))
#define GPIO_PortC_ODRControl ((volatile GPIO_ODR_Register*)(PORTC_BASE +  0x14))
#define GPIO_PortD_ODRControl ((volatile GPIO_ODR_Register*)(PORTD_BASE +  0x14))
#define GPIO_PortE_ODRControl ((volatile GPIO_ODR_Register*)(PORTE_BASE +  0x14))
#define GPIO_PortF_ODRControl ((volatile GPIO_ODR_Register*)(PORTF_BASE +  0x14))

#define GPIO_PortA_ModeSelect ((volatile GPIO_MODER_Register*)(PORTA_BASE +  0x00))
#define GPIO_PortB_ModeSelect ((volatile GPIO_MODER_Register*)(PORTB_BASE +  0x00))
#define GPIO_PortC_ModeSelect ((volatile GPIO_MODER_Register*)(PORTC_BASE +  0x00))
#define GPIO_PortD_ModeSelect ((volatile GPIO_MODER_Register*)(PORTD_BASE +  0x00))
#define GPIO_PortE_ModeSelect ((volatile GPIO_MODER_Register*)(PORTE_BASE +  0x00))
#define GPIO_PortF_ModeSelect ((volatile GPIO_MODER_Register*)(PORTF_BASE +  0x00))

#define GPIO_PortA_AlternateFunctionLowSelect ((volatile GPIO_AFRL_Register*)(PORTA_BASE +  0x20))
#define GPIO_PortB_AlternateFunctionLowSelect ((volatile GPIO_AFRL_Register*)(PORTB_BASE +  0x20))
#define GPIO_PortC_AlternateFunctionLowSelect ((volatile GPIO_AFRL_Register*)(PORTC_BASE +  0x20))
#define GPIO_PortD_AlternateFunctionLowSelect ((volatile GPIO_AFRL_Register*)(PORTD_BASE +  0x20))
#define GPIO_PortE_AlternateFunctionLowSelect ((volatile GPIO_AFRL_Register*)(PORTE_BASE +  0x20))
#define GPIO_PortF_AlternateFunctionLowSelect ((volatile GPIO_AFRL_Register*)(PORTF_BASE +  0x20))

#define GPIO_PortA_AlternateFunctionHighSelect ((volatile GPIO_AFRH_Register*)(PORTA_BASE +  0x24))
#define GPIO_PortB_AlternateFunctionHighSelect ((volatile GPIO_AFRH_Register*)(PORTB_BASE +  0x24))
#define GPIO_PortC_AlternateFunctionHighSelect ((volatile GPIO_AFRH_Register*)(PORTC_BASE +  0x24))
#define GPIO_PortD_AlternateFunctionHighSelect ((volatile GPIO_AFRH_Register*)(PORTD_BASE +  0x24))
#define GPIO_PortE_AlternateFunctionHighSelect ((volatile GPIO_AFRH_Register*)(PORTE_BASE +  0x24))
#define GPIO_PortF_AlternateFunctionHighSelect ((volatile GPIO_AFRH_Register*)(PORTF_BASE +  0x24))

typedef struct
{
    uint32_t GPIO_PortIndex;        /*!< Specifies the GPIO pin number. */
    uint32_t GPIO_PinNumber;        /*!< Specifies the GPIO pin number. */
    uint32_t GPIO_PinMode;          /*!< Specifies the mode of the GPIO pin. */
    uint32_t GPIO_PinSpeed;         /*!< Specifies the speed of the GPIO pin. */
    uint32_t GPIO_PinPuPdControl;   /*!< Specifies the pull-up/pull-down configuration for the GPIO pin. */
    uint32_t GPIO_PinPinOPType;     /*!< Specifies the output type of the GPIO pin. */
    uint32_t GPIO_PinAltFunMode;    /*!< Specifies the alternate function mode of the GPIO pin. */
} GPIO_PinConfig_t;

// grouping the array of pointers in one structure
typedef struct {
    volatile GPIO_IDR_Register *const GPIO_IDRControl_Registers[Port_Indices];
    volatile GPIO_ODR_Register *const GPIO_ODRControl_Registers[Port_Indices];
    volatile GPIO_MODER_Register *const GPIO_ModeSelect_Registers[Port_Indices];
    volatile GPIO_AFRL_Register *const GPIO_AlternateFunctionLowSelect_Registers[Port_Indices];
    volatile GPIO_AFRH_Register *const GPIO_AlternateFunctionHighSelect_Registers[Port_Indices];
    GPIO_PinConfig_t GPIO_PinConfig; /*!< Holds GPIO pin configuration settings. */
} GPIORegisters;

#endif /* GPIO_REG_H_ */

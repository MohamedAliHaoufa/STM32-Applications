/**
 * @file stm32f407xx.h
 * @author Your Name
 * @brief Header file containing all the necessary information about the STM32F407xx MCU.
 * @version 0.1
 * @date 2023-08-10
 *
 * This file contains the definitions and macros for the STM32F407xx microcontroller
 * peripherals and memory maps.
 */

/** @defgroup STM32F407XX_HEADER STM32F407xx MCU Header File
 * @{
 * @brief Header file containing all the necessary information about the STM32F407xx MCU.
 */
#ifndef INC_MCAL_STM32F407XX_H_
#define INC_MCAL_STM32F407XX_H_

#include "../MCAL_Layer/mcal_std_types.h"
#include "../MCAL_Layer/std_libraries.h"

/**
 * @defgroup STM32F407XX_DEFINITIONS Definitions and Macros
 * @{
 * @brief Various definitions and macros for the STM32F407xx MCU.
 */

/** @brief volatile_32bit_register Define for accessing a volatile 32-bit register */
#define __vo    volatile

/** @brief Define for the weak attribute */
#define __weak  __attribute__((weak))

/** @} STM32F407XX_DEFINITIONS */


/********************************** START : Processor Specific Details ***************************************************/

/**
 * @defgroup NVIC_ISERx NVIC ISERx Registers
 * @{
 * @brief Base addresses for NVIC ISERx registers.
 */
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)  
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)  
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)  
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4      	((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5        	((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6      	((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7      	((__vo uint32_t*)0xE000E11C)
/** @} NVIC_ISERx */

/**
 * @defgroup NVIC_ICERx NVIC ICERx Registers
 * @{
 * @brief Base addresses for NVIC ICERx registers.
 */
#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180)  
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184)  
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188)  
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C)  
#define NVIC_ICER4		    ((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5		    ((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6  		((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7		    ((__vo uint32_t*)0xE000E19C)
/** @} NVIC_ICERx */

/********************************** END : Processor Specific Details ***************************************************/

/********************************** START : Memory Base Addresses ***************************************************/

/**
 * @defgroup NVIC_PR_BASE NVIC Priority Registers Base Address
 * @{
 * @brief Base address for NVIC Priority registers calculation.
 */

#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400)

/** @} NVIC_PR_BASE */

/**
 * @defgroup PR_BITS_IMPLEMENTED Priority Bits Implemented
 * @{
 * @brief Number of Priority bits implemented in the priority register.
 */

#define NO_PR_BITS_IMPLEMENTED      4

/** @} PR_BITS_IMPLEMENTED */

/**
 * @defgroup MEMORY_BASE_ADDRESSES Memory Base Addresses
 * @{
 * @brief Base addresses of memory regions.
 */
#define FLASH_BASEADDR               0x08000000U   /**< Base address of FLASH memory */
#define SRAM1_BASEADDR     (uint32_t)0x20000000    /**< Base address of SRAM1 memory */
#define SRAM                         SRAM1_BASE_ADDR

#define ROM_BASEADDR                 0x1FFF0000U   /**< Base address of ROM memory */
#define SRAM2_BASEADDR               0x2001C000U   /**< Base address of SRAM2 memory */

/** @} MEMORY_BASE_ADDRESSES */
/********************************** END : Memory Base Addresses ***************************************************/

/**
 * @defgroup STM32F407XX_PERIPHERALS Peripheral Base Addresses
 * @{
 * @brief Base addresses of various peripherals for the STM32F407xx MCU.
 */

/********************************** START : Peripheral Base Addresses ***************************************************/

/**
 * @defgroup Peripheral_Base_Addresses Peripheral Base Addresses
 * @{
 * @brief Base addresses of AHBx and APBx bus peripherals.
 */
#define PERIPH_BASEADDR            0x40000000U	   /**< Base address of peripheral memory */
#define APB1PERIPH_BASEADDR        PERIPH_BASEADDR /**< Base address of APB1 peripheral memory */
#define APB2PERIPH_BASEADDR        0x40010000U	   /**< Base address of APB2 peripheral memory */
#define AHB1PERIPH_BASEADDR        0x40020000U	   /**< Base address of AHB1 peripheral memory */
#define AHB2PERIPH_BASEADDR        0x50000000U	   /**< Base address of AHB2 peripheral memory */

/** @} Peripheral_Base_Addresses */
/********************************** END : Peripheral Base Addresses ***************************************************/

/********************************** START : Peripheral Base Addresses on APB1 Bus ***************************************************/

/**
 * @defgroup Peripheral_Base_Addresses_APB1 Peripheral Base Addresses on APB1 Bus
 * @{
 * @brief Base addresses of peripherals connected to the APB1 bus.
 */
#define GPIOA_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0000)    /**< Base address of GPIOA peripheral */
#define GPIOB_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0400)    /**< Base address of GPIOB peripheral */
#define GPIOC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0800)    /**< Base address of GPIOC peripheral */
#define GPIOD_BASEADDR        (AHB1PERIPH_BASEADDR + 0x0C00)    /**< Base address of GPIOD peripheral */
#define GPIOE_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1000)    /**< Base address of GPIOE peripheral */
#define GPIOF_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1400)    /**< Base address of GPIOF peripheral */
#define GPIOG_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1800)    /**< Base address of GPIOG peripheral */
#define GPIOH_BASEADDR        (AHB1PERIPH_BASEADDR + 0x1C00)    /**< Base address of GPIOH peripheral */
#define GPIOI_BASEADDR        (AHB1PERIPH_BASEADDR + 0x2000)    /**< Base address of GPIOI peripheral */
#define RCC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x3800)    /**< Base address of RCC peripheral */
#define DMA1_BASEADDR         (AHB1PERIPH_BASEADDR + 0x6000)    /**< Base address of DMA1 peripheral */
#define DMA2_BASEADDR         (AHB1PERIPH_BASEADDR + 0x6400)    /**< Base address of DMA2 peripheral */
#define CRC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x3000)    /**< Base address of CRC peripheral */
#define FIR_BASEADDR          (AHB1PERIPH_BASEADDR + 0x3C00)    /**< Base address of Flash Interface peripheral */

#define I2C1_BASEADDR         (APB1PERIPH_BASEADDR+0x5400)  /**< Base address of I2C1 peripheral */
#define I2C2_BASEADDR         (APB1PERIPH_BASEADDR+0x5800)  /**< Base address of I2C2 peripheral */
#define I2C3_BASEADDR         (APB1PERIPH_BASEADDR+0x5C00)  /**< Base address of I2C3 peripheral */

#define SPI2_BASEADDR         (APB1PERIPH_BASEADDR+0x3800)  /**< Base address of SPI2 peripheral */
#define SPI3_BASEADDR         (APB1PERIPH_BASEADDR+0x3C00)  /**< Base address of SPI3 peripheral */

#define USART2_BASEADDR       (APB1PERIPH_BASEADDR+0x4400)  /**< Base address of USART2 peripheral */
#define USART3_BASEADDR       (APB1PERIPH_BASEADDR+0x4800)  /**< Base address of USART3 peripheral */
#define UART4_BASEADDR        (APB1PERIPH_BASEADDR+0x4C00)  /**< Base address of UART4 peripheral */
#define UART5_BASEADDR        (APB1PERIPH_BASEADDR+0x5000)  /**< Base address of UART5 peripheral */

/** @} Peripheral_Base_Addresses_APB1 */
/********************************** END : Peripheral Base Addresses on APB1 Bus ***************************************************/

/********************************** START : Peripheral Base Addresses on APB2 Bus ***************************************************/
/**
 * @defgroup Peripheral_Base_Addresses_APB2 Peripheral Base Addresses on APB2 Bus
 * @{
 * @brief Base addresses of peripherals connected to the APB2 bus.
 */
#define SPI1_BASEADDR         (APB2PERIPH_BASEADDR+0x3000)  /**< Base address of SPI1 peripheral */
#define SPI4_BASEADDR         (APB2PERIPH_BASEADDR+0x3400)  /**< Base address of SPI4 peripheral */
#define USART1_BASEADDR       (APB2PERIPH_BASEADDR+0x1000)  /**< Base address of USART1 peripheral */
#define USART6_BASEADDR       (APB2PERIPH_BASEADDR+0x1400)  /**< Base address of USART6 peripheral */
#define SYSCFG_BASEADDR       (APB2PERIPH_BASEADDR+0x3800)  /**< Base address of SYSCFG peripheral */
#define EXTI_BASEADDR         (APB2PERIPH_BASEADDR+0x3C00)  /**< Base address of EXTI peripheral */
/** @} Peripheral_Base_Addresses_APB2 */

/** @} STM32F407XX_PERIPHERALS */
/********************************** END : Peripheral Base Addresses on APB2 Bus ***************************************************/

/********************************** START : Peripheral register definition structure ***************************************************/
/**
 * @defgroup Peripheral_Registers Peripheral Register Definitions
 * @{
 * @brief Structures defining the register layouts for various peripherals.
 */

/**
 * @struct GPIO_RegDef_t
 * @brief GPIO peripheral register definition structure
 */
typedef struct {
    __vo uint32_t MODER;      /**< GPIO port mode register */
    __vo uint32_t OTYPER;     /**< GPIO port output type register */
    __vo uint32_t OSPEEDR;    /**< GPIO port output speed register */
    __vo uint32_t PUPDR;      /**< GPIO port pull-up/pull-down register */
    __vo uint32_t IDR;        /**< GPIO port input data register */
    __vo uint32_t ODR;        /**< GPIO port output data register */
    __vo uint32_t BSRR;       /**< GPIO port bit set/reset register */
    __vo uint32_t LCKR;       /**< GPIO port configuration lock register */
    __vo uint32_t AFR[2];     /**< GPIO alternate function registers (AFR[0] = low, AFR[1] = high) */
} GPIO_RegDef_t;

/**
 * @struct RCC_RegDef_t
 * @brief RCC peripheral register definition structure
 */
typedef struct {
    __vo uint32_t CR;         /**< RCC control register */
    __vo uint32_t PLLCFGR;    /**< RCC PLL configuration register */
    __vo uint32_t CFGR;       /**< RCC configuration register */
    __vo uint32_t CIR;        /**< RCC clock configuration register */
    __vo uint32_t AHB1RSTR;   /**< RCC AHB1 peripheral reset register */
    __vo uint32_t AHB2RSTR;   /**< RCC AHB2 peripheral reset register */
    __vo uint32_t AHB3RSTR;   /**< RCC AHB3 peripheral reset register */
    uint32_t      RESERVED0;
    __vo uint32_t APB1RSTR;   /**< RCC APB1 peripheral reset register */
    __vo uint32_t APB2RSTR;   /**< RCC APB2 peripheral reset register */
    uint32_t      RESERVED1[2];
    __vo uint32_t AHB1ENR;    /**< RCC AHB1 peripheral clock enable register */
    __vo uint32_t AHB2ENR;    /**< RCC AHB2 peripheral clock enable register */
    __vo uint32_t AHB3ENR;    /**< RCC AHB3 peripheral clock enable register */
    uint32_t      RESERVED2;
    __vo uint32_t APB1ENR;    /**< RCC APB1 peripheral clock enable register */
    __vo uint32_t APB2ENR;    /**< RCC APB2 peripheral clock enable register */
    uint32_t      RESERVED3[2];
    __vo uint32_t AHB1LPENR;  /**< RCC AHB1 peripheral clock enable in low power mode register */
    __vo uint32_t AHB2LPENR;  /**< RCC AHB2 peripheral clock enable in low power mode register */
    __vo uint32_t AHB3LPENR;  /**< RCC AHB3 peripheral clock enable in low power mode register */
    uint32_t      RESERVED4;
    __vo uint32_t APB1LPENR;  /**< RCC APB1 peripheral clock enable in low power mode register */
    __vo uint32_t APB2LPENR;  /**< RCC APB2 peripheral clock enable in low power mode register */
    uint32_t      RESERVED5[2];
    __vo uint32_t BDCR;       /**< RCC backup domain control register */
    __vo uint32_t CSR;        /**< RCC control/status register */
    uint32_t      RESERVED6[2];
    __vo uint32_t SSCGR;      /**< RCC spread spectrum clock generation register */
    __vo uint32_t PLLI2SCFGR; /**< RCC PLLI2S configuration register */
    __vo uint32_t PLLSAICFGR; /**< RCC PLLSAI configuration register */
    __vo uint32_t DCKCFGR;    /**< RCC dedicated clock configuration register */
    __vo uint32_t CKGATENR;   /**< RCC clocks gated enable register */
    __vo uint32_t DCKCFGR2;   /**< RCC dedicated clocks configuration register 2 */
} RCC_RegDef_t;

/**
 * @struct EXTI_RegDef_t
 * @brief EXTI peripheral register definition structure
 */
typedef struct {
    __vo uint32_t IMR;      /**< EXTI interrupt mask register */
    __vo uint32_t EMR;      /**< EXTI event mask register */
    __vo uint32_t RTSR;     /**< EXTI rising trigger selection register */
    __vo uint32_t FTSR;     /**< EXTI falling trigger selection register */
    __vo uint32_t SWIER;    /**< EXTI software interrupt event register */
    __vo uint32_t PR;       /**< EXTI pending register */
} EXTI_RegDef_t;

/**
 * @struct SYSCFG_RegDef_t
 * @brief SYSCFG peripheral register definition structure
 */
typedef struct {
    __vo uint32_t MEMRMP;    /**< SYSCFG memory remap register */
    __vo uint32_t PMC;       /**< SYSCFG peripheral mode configuration register */
    __vo uint32_t EXTICR[4]; /**< SYSCFG external interrupt configuration registers */
    uint32_t      RESERVED1[2];
    __vo uint32_t CMPCR;
    uint32_t      RESERVED2[2];
    __vo uint32_t CFGR;
} SYSCFG_RegDef_t;

/**
 * @struct I2C_RegDef_t
 * @brief I2C peripheral register definition structure
 */
typedef struct {
    __vo uint32_t CR1;       /**< I2C control register 1 */
    __vo uint32_t CR2;       /**< I2C control register 2 */
    __vo uint32_t OAR1;      /**< I2C own address register 1 */
    __vo uint32_t OAR2;      /**< I2C own address register 2 */
    __vo uint32_t DR;        /**< I2C data register */
    __vo uint32_t SR1;       /**< I2C status register 1 */
    __vo uint32_t SR2;       /**< I2C status register 2 */
    __vo uint32_t CCR;       /**< I2C clock control register */
    __vo uint32_t TRISE;     /**< I2C rise time register */
    __vo uint32_t FLTR;      /**< I2C digital filter register */
} I2C_RegDef_t;

/** @} Peripheral_Registers */
/********************************** END : Peripheral register definition structure ***************************************************/

/********************************** START : Peripheral Definitions ***************************************************/

/**
 * @defgroup Peripheral_Definitions Peripheral Definitions
 * @{
 * @brief Definitions for various peripheral instances based on their base addresses.
 */
#define  GPIOA       ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define  GPIOB       ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define  GPIOC       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define  GPIOD       ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define  GPIOE       ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define  GPIOF       ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define  GPIOG       ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define  GPIOH       ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define  GPIOI       ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC          ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI         ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG       ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define I2C1         ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2         ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3         ((I2C_RegDef_t*)I2C3_BASEADDR)

/** @} Peripheral Definitions */
/********************************** END : Peripheral Definitions ***************************************************/

/********************************** START : Clock Enable Macros ***************************************************/
/**
 * @defgroup Clock_Enable_Macros Clock Enable Macros
 * @{
 */

/**
 * @defgroup Clock_Enable_GPIO_Clocks Clock Enable GPIO Clocks
 * @{
 * @brief Macros for enabling clocks to GPIO peripherals.
 */
#define GPIOA_PCLK_EN()      ( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()      ( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()      ( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()      ( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()      ( RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN()      ( RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN()      ( RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN()      ( RCC->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN()      ( RCC->AHB1ENR |= (1<<8) )
/** @} Clock_Enable_GPIO_Clocks */

/**
 * @defgroup Clock_Enable_I2C_Clocks Clock Enable I2C Clocks
 * @{
 * @brief Macros for enabling clocks to I2C peripherals.
 */ 
#define I2C1_PCLK_EN()      ( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()      ( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()      ( RCC->APB1ENR |= (1<<23) )
/** @} Clock_Enable_I2C_Clocks */

/**
 * @defgroup Clock_Enable_SPI_Clocks Clock Enable SPI Clocks
 * @{
 * @brief Macros for enabling clocks to SPI peripherals.
 */
#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |= (1<<15) )
/** @} Clock_Enable_SPI_Clocks */

/**
 * @defgroup Clock_Enable_USART_Clocks Clock Enable USART Clocks
 * @{
 * @brief Macros for enabling clocks to USART peripherals.
 */
#define USART2_PCLK_EN()      ( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()      ( RCC->APB1ENR |= (1<<18) )
#define USART1_PCLK_EN()      ( RCC->APB2ENR |= (1<<4) )
#define USART6_PCLK_EN()      ( RCC->APB2ENR |= (1<<5) )
/** @} Clock_Enable_USART_Clocks */

/**
 * @defgroup Clock_Enable_SYSCFG_Clocks Clock Enable SYSCFG Clocks
 * @{
 * @brief Macros for enabling clocks to SYSCFG peripherals.
 */
#define SYSCFG_PCLK_EN()      ( RCC->APB2ENR |= (1<<14) )
/** @} Clock_Enable_SYSCFG_Clocks */

/** @} Clock_Enable_Macros */
/********************************** END : Clock Enable Macros ***************************************************/

/********************************** START : Clock Disable Macros ***************************************************/
/**
 * @defgroup Clock_Disable_Macros Clock Disable Macros
 * @{
 */

/**
 * @defgroup Disable_clock_for_GPIOx Disable clock for GPIOx peripherals
 * @{
 */
#define GPIOA_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<0) )
#define GPIOB_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<1) )
#define GPIOC_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<2) )
#define GPIOD_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<3) )
#define GPIOE_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<4) )
#define GPIOF_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<5) )
#define GPIOG_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<6) )
#define GPIOH_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<7) )
#define GPIOI_PCLK_DI()      ( RCC->AHB1ENR &= ~(uint32_t)(1<<8) )
/** @} Disable_clock_for_GPIOx */

/**
 * @defgroup Disable_clock_for_I2Cx Disable clock for I2Cx peripherals
 * @{
 */
#define I2C1_PCLK_DI()      ( RCC->APB1ENR &= ~(uint32_t)(1<<21) )
#define I2C2_PCLK_DI()      ( RCC->APB1ENR &= ~(uint32_t)(1<<22) )
#define I2C3_PCLK_DI()      ( RCC->APB1ENR &= ~(uint32_t)(1<<23) )
/** @} Disable_clock_for_I2Cx */

/**
 * @defgroup Disable_clock_for_SPIx Disable clock for SPIx peripherals
 * @{
 */
#define SPI1_PCLK_DI()      ( RCC->APB2ENR &= ~(uint32_t)(1<<12) )
#define SPI2_PCLK_DI()      ( RCC->APB1ENR &= ~(uint32_t)(1<<14) )
#define SPI3_PCLK_DI()      ( RCC->APB1ENR &= ~(uint32_t)(1<<15) )
/** @} Disable_clock_for_SPIx */

/**
 * @defgroup Disable_clock_for_USARTx Disable clock for USARTx peripherals
 * @{
 */
#define USART2_PCLK_DI()      ( RCC->APB1ENR &= ~(uint32_t)(1<<17) )
#define USART3_PCLK_DI()      ( RCC->APB1ENR &= ~(uint32_t)(1<<18) )
#define USART1_PCLK_DI()      ( RCC->APB2ENR &= ~(uint32_t)(1<<4) )
#define USART6_PCLK_DI()      ( RCC->APB2ENR &= ~(uint32_t)(1<<5) )
/** @} Disable_clock_for_USARTx */

/**
 * @defgroup Disable_clock_for_SYSCFG Disable clock for SYSCFG peripherals
 * @{
 */
#define SYSCFG_PCLK_DI()      ( RCC->APB2ENR &= ~(1<<14) )
/** @} Disable_clock_for_SYSCFG */

/** @} Clock_Disable_Macros */
/********************************** END : Clock Disable Macros ***************************************************/

/********************************** START : Reset peripherals Macros ***************************************************/
/**
 * @defgroup Reset_Peripherals_Macros Reset Peripherals Macros
 * @{
 * @brief Macros for resetting various peripherals.
 */

/**
 * @defgroup Reset_GPIOx_Reset Reset GPIOx Peripherals
 * @{
 * @brief Macros for resetting GPIO peripherals.
 */
#define GPIOA_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<0) ); ( RCC->AHB1RSTR  &= ~(1<<0) );}while(0)
#define GPIOB_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<1) ); ( RCC->AHB1RSTR  &= ~(1<<1) );}while(0)
#define GPIOC_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<2) ); ( RCC->AHB1RSTR  &= ~(1<<2) );}while(0)
#define GPIOD_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<3) ); ( RCC->AHB1RSTR  &= ~(1<<3) );}while(0)
#define GPIOE_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<4) ); ( RCC->AHB1RSTR  &= ~(1<<4) );}while(0)
#define GPIOF_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<5) ); ( RCC->AHB1RSTR  &= ~(1<<5) );}while(0)
#define GPIOG_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<6) ); ( RCC->AHB1RSTR  &= ~(1<<6) );}while(0)
#define GPIOH_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<7) ); ( RCC->AHB1RSTR  &= ~(1<<7) );}while(0)
#define GPIOI_REG_RESET()     do{ ( RCC->AHB1RSTR |= (1<<8) ); ( RCC->AHB1RSTR  &= ~(1<<8) );}while(0)
/** @} Reset_GPIOx_Reset */

/**
 * @defgroup Reset_I2Cx_Reset Reset I2Cx Peripherals
 * @{
 * @brief Macros for resetting I2C peripherals.
 */
#define I2C1_REG_RESET()     do{ ( RCC->APB1RSTR |= (1<<21) ); ( RCC->APB1RSTR  &= ~(1<<21) );}while(0)
#define I2C2_REG_RESET()     do{ ( RCC->APB1RSTR |= (1<<22) ); ( RCC->APB1RSTR  &= ~(1<<22) );}while(0)
#define I2C3_REG_RESET()     do{ ( RCC->APB1RSTR |= (1<<23) ); ( RCC->APB1RSTR  &= ~(1<<23) );}while(0)
/** @} Reset_I2Cx_Reset */

/** @} Reset_Peripherals_Macros */
/********************************** END : Reset peripherals Macros ***************************************************/

/**
 * @defgroup GPIO_Base_Address_to_Code GPIO Base Address to Code Conversion Macros
 * @{
 * @brief Macros for converting GPIO base addresses to corresponding port codes.
 */

/**
 * @brief Macro to convert GPIO base address to port code.
 *
 * @param x GPIOx base address.
 * @return Corresponding port code for the given GPIO base address.
 */
#define GPIO_BASEADDR_TO_CODE(x)  (   (x == GPIOA)?0:\
                                      (x == GPIOB)?1:\
                                      (x == GPIOC)?2:\
                                      (x == GPIOD)?3:\
                                      (x == GPIOE)?4:\
                                      (x == GPIOF)?5:\
                                      (x == GPIOG)?6:\
                                      (x == GPIOH)?7:\
                                      (x == GPIOI)?8:0 )

/** @} GPIO_Base_Address_to_Code */

/********************************** START : IRQ (interrupt request) Numbers Macros ***************************************************/

/**
 * @defgroup IRQ_Numbers_Macros IRQ Numbers Macros
 * @{
 * @brief Macros for interrupt request numbers and priority levels.
 */

/**
 * @defgroup IRQ_Numbers EXTI (External Interrupt) IRQ Numbers
 * @{
 * @brief IRQ numbers for EXTI (External Interrupt) sources.
 */
#define IRQ_NO_EXTI0          6
#define IRQ_NO_EXTI1          7 
#define IRQ_NO_EXTI2          8
#define IRQ_NO_EXTI3          9
#define IRQ_NO_EXTI4          10
#define IRQ_NO_EXTI9_5        23
#define IRQ_NO_EXTI15_10      40
/** @} IRQ_Numbers EXTI (External Interrupt) IRQ Numbers */

/**
 * @defgroup IRQ_Numbers SPI IRQ Numbers
 * @{
 * @brief IRQ numbers for SPI sources.
 */
#define IRQ_NO_SPI1           35
#define IRQ_NO_SPI2           36
#define IRQ_NO_SPI3           51
/** @} IRQ_Numbers SPI IRQ Numbers */

/**
 * @defgroup IRQ_Numbers I2C IRQ Numbers
 * @{
 * @brief IRQ numbers for I2C sources.
 */
#define IRQ_NO_I2C1_EV        31 
#define IRQ_NO_I2C1_ER        32
#define IRQ_NO_I2C2_EV        33
#define IRQ_NO_I2C2_ER        34
#define IRQ_NO_I2C3_EV        79
#define IRQ_NO_I2C3_ER        80
/** @} IRQ_Numbers I2C IRQ Numbers */

/** @} IRQ_Numbers_Macros */
/********************************** END : IRQ (interrupt request) Numbers Macros ***************************************************/

/********************************** START : ALL NVIC possible priority levels Macros ***************************************************/

/**
 * @defgroup NVIC_Priority_Levels NVIC Priority Levels Macros
 * @{
 * @brief Macros for all possible priority levels for NVIC.
 */
#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI1     1
#define NVIC_IRQ_PRI2     2
#define NVIC_IRQ_PRI3     3
#define NVIC_IRQ_PRI4     4
#define NVIC_IRQ_PRI5     5
#define NVIC_IRQ_PRI6     6
#define NVIC_IRQ_PRI7     7
#define NVIC_IRQ_PRI8     8
#define NVIC_IRQ_PRI9     9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15
/** @} NVIC_Priority_Levels */

/********************************** END : ALL NVIC possible priority levels Macros ***************************************************/

/********************************** START : I2C Bit position definitions Macros ***************************************************/

/**
 * @defgroup I2C_Bit_Positions I2C Bit Position Definitions
 * @{
 * @brief Bit position definitions for various registers in the I2C peripheral.
 */

/** 
 * @defgroup I2C_CR1_Bit_Positions I2C_CR1 Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_CR1 register.
 */
#define I2C_CR1_PE           0    /**< Peripheral Enable */
#define I2C_CR1_SMBUS        1    /**< SMBus Mode */
#define I2C_CR1_SMBTYPE      3    /**< SMBus Type */
#define I2C_CR1_ENARP        4    /**< ARP Enable */
#define I2C_CR1_ENPEC        5    /**< PEC Enable */
#define I2C_CR1_ENGC         6    /**< General Call Enable */
#define I2C_CR1_NOSTRETCH    7    /**< Clock Stretching Disable */
#define I2C_CR1_START        8    /**< Start Generation */
#define I2C_CR1_STOP         9    /**< Stop Generation */
#define I2C_CR1_ACK          10   /**< Acknowledge Enable */
#define I2C_CR1_POS          11   /**< Acknowledge/Not Acknowledge */
#define I2C_CR1_PEC          12   /**< Packet Error Checking */
#define I2C_CR1_ALERT        13   /**< SMBus Alert */
#define I2C_CR1_SWRST        15   /**< Software Reset */
/** @} I2C_CR1_Bit_Positions */

/** 
 * @defgroup I2C_OAR1_Bit_Positions I2C_OAR1 Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_OAR1 register.
 */
#define I2C_OAR1_ADD0        0    /**< Addressing Mode Bit 0 */
#define I2C_OAR1_ADD         1    /**< Interface Address */
#define I2C_OAR1_ADDMODE     15   /**< Addressing Mode */
/** @} I2C_OAR1_Bit_Positions */

/** 
 * @defgroup I2C_OAR2_Bit_Positions I2C_OAR2 Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_OAR2 register.
 */
#define I2C_OAR2_ENDUAL      0    /**< Dual Addressing Mode Enable */
#define I2C_OAR2_ADD2        1    /**< Interface Address 2 */
/** @} I2C_OAR2_Bit_Positions */

/* 
 * Bit position definitions for I2C_DR
 * Note: Bit positions for I2C_DR are missing in the provided content
 */

/** 
 * @defgroup I2C_CR2_Bit_Positions I2C_CR2 Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_CR2 register.
 */
#define I2C_CR2_FREQ      0    /**< Peripheral Clock Frequency */
#define I2C_CR2_ITERREN   8    /**< Error Interrupt Enable */
#define I2C_CR2_ITEVTEN   9    /**< Event Interrupt Enable */
#define I2C_CR2_ITBUFEN   10   /**< Buffer Interrupt Enable */
#define I2C_CR2_DMAEN     11   /**< DMA Requests Enable */
#define I2C_CR2_LAST      12   /**< DMA Last Transfer */
/** @} I2C_CR2_Bit_Positions */

/** 
 * @defgroup I2C_SR1_Bit_Positions I2C_SR1 Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_SR1 register.
 */
#define I2C_SR1_SB        0    /**< Start Bit */
#define I2C_SR1_ADDR      1    /**< Address Sent (master mode) / Address Matched (slave mode) */
#define I2C_SR1_BTF       2    /**< Byte Transfer Finished */
#define I2C_SR1_ADD10     3    /**< 10-bit Header Sent (Master mode) */
#define I2C_SR1_STOPF     4    /**< Stop Detection (Slave mode) */
#define I2C_SR1_RxNE      6    /**< Data Register Not Empty (receivers) */
#define I2C_SR1_TxE       7    /**< Data Register Empty (transmitters) */
#define I2C_SR1_BERR      8    /**< Bus Error */
#define I2C_SR1_ARLO      9    /**< Arbitration Lost (master mode) */
#define I2C_SR1_AF        10   /**< Acknowledge Failure */
#define I2C_SR1_OVR       11   /**< Overrun/Underrun */
#define I2C_SR1_PECERR    12   /**< PEC Error in reception */
#define I2C_SR1_TIMEOUT   14   /**< Timeout or Tlow Error */
#define I2C_SR1_SMBALERT  15   /**< SMBus Alert */
/** @} I2C_SR1_Bit_Positions */

/** 
 * @defgroup I2C_SR2_Bit_Positions I2C_SR2 Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_SR2 register.
 */
#define I2C_SR2_MSL         0    /**< Master/Slave */
#define I2C_SR2_BUSY        1    /**< Bus Busy */
#define I2C_SR2_TRA         2    /**< Transmitter/Receiver */
#define I2C_SR2_GENCALL     4    /**< General Call Address (Slave mode) */
#define I2C_SR2_SMBDEFAULT  5    /**< SMBus Device Default Address (Slave mode) */
#define I2C_SR2_SMBHOST     6    /**< SMBus Host Header (Slave mode) */
#define I2C_SR2_DUALF       7    /**< Dual Flag (Slave mode) */
#define I2C_SR2_PEC         8    /**< Packet Error Checking Register */
/** @} I2C_SR2_Bit_Positions */

/** 
 * @defgroup I2C_CCR_Bit_Positions I2C_CCR Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_CCR register.
 */
#define I2C_CCR_CCR         0    /**< Clock Control Register in Fast/Standard mode */
#define I2C_CCR_DUTY        14   /**< Fast Mode Duty Cycle */
#define I2C_CCR_FS          15   /**< I2C Master Mode Selection */
/** @} I2C_CCR_Bit_Positions */

/** 
 * @defgroup I2C_TRISE_Bit_Positions I2C_TRISE Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_TRISE register.
 */
#define I2C_TRISE_TRISE     0    /**< Maximum Rise Time in Fast/Standard mode */
/** @} I2C_TRISE_Bit_Positions */

/** 
 * @defgroup I2C_FLTR_Bit_Positions I2C_FLTR Bit Position Definitions
 * @{
 * @brief Bit position definitions for I2C_FLTR register.
 */
#define I2C_FLTR_DNF        0    /**< Digital Noise Filter */
#define I2C_FLTR_ANOFF      4    /**< Analog Noise Filter */
/** @} I2C_FLTR_Bit_Positions */

/** @} I2C_Bit_Positions */
/********************************** END : I2C Bit position definitions Macros ***************************************************/

#include "../MCAL_Layer/RCC/hal_rcc.h"    		  	  /**< Include RCC driver header file */
#include "../MCAL_Layer/GPIO/hal_gpio.h" 		  		  /**< Include GPIO driver header file */

//#include "../MCAL/I2C/I2C.h"  		  /**< Include I2C driver header file */
//#include "../MCAL/SPI/SPI.h" 	 	      /**< Include SPI driver header file */
//#include "../MCAL/UART/UART.h"  		  /**< Include UART driver header file */
//#include "../MCAL/USART/USART.h"  	  /**< Include USART driver header file */
//#include "../MCAL/NVIC/NVIC.h"   		  /**< Include NVIC driver header file */
//#include "../MCAL/DMA/DMA.h"  	      /**< Include DMA driver header file */
//#include "../MCAL/TIM/TIM.h"    		  /**< Include TIM driver header file */
//#include "../MCAL/CAN/CAN.h"      	  /**< Include CAN driver header file */
//#include "../MCAL/Ethernet/Ethernet.h"  /**< Include Ethernet driver header file */
//#include "../MCAL/RTC/RTC.h" 			  /**< Include RTC driver header file */
//#include "../MCAL/ADC/ADC.h"  		  /**< Include ADC driver header file */
//#include "../MCAL/DAC/DAC.h" 			  /**< Include DAC driver header file */
//#include "../MCAL/I2S/I2S.h" 			  /**< Include I2S driver header file */

/** @} STM32F407XX_HEADER*/

#endif /* INC_MCAL_STM32F407XX_H_ */











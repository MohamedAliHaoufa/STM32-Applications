/**
 * @file hal_gpio.c
 * @author your name (you@domain.com)
 * @brief This file contains the GPIO driver implementation
 *
 * This file provides the implementation of a General-Purpose Input/Output (GPIO) driver
 * for a specific hardware platform or microcontroller. It allows users to configure,
 * control, and interact with GPIO pins to perform various tasks, such as input reading,
 * output writing, and interrupt handling.
 *
 * @version 0.1
 * @date 2023-08-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "hal_gpio.h"

/**
 * @addtogroup GPIO_Private_Functions Private Functions
 * @{
 */

/**
 * @brief Array of pointers to hardware registers.
 * Application layer should not access these directly, use interfacing functions defined in GPIO.h.
 */
volatile EXTI_RegDef_t static *const pEXTI_RegisterControl[1] = {
    EXTI_RegisterControl
};

volatile SYSCFG_RegDef_t static *const pSYSCFG_RegisterControl[1] = {
    SYSCFG_RegisterControl
};

volatile GPIO_RegDef_t static *const pGPIO_Portx_RegisterControl[Port_Indices] = {
    GPIO_PortA_RegisterControl,
    GPIO_PortB_RegisterControl,
    GPIO_PortC_RegisterControl,
    GPIO_PortD_RegisterControl,
    GPIO_PortE_RegisterControl,
    GPIO_PortF_RegisterControl
};


Std_ReturnType EXTI_EnableDisableInterrupt(uint8_t PinNum, uint8_t EnorDi) {

	Std_ReturnType ret = E_OK;
    if (pEXTI_RegisterControl[0] != NULL) {
        switch (EnorDi) {
            case ENABLE:
            	pEXTI_RegisterControl[0]->IMR |= (1<<PinNum);
                break;
            case DISABLE:
            	pEXTI_RegisterControl[0]->IMR &= (1<<PinNum);
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

/**
 * @brief A simple delay function.
 */
void Delay(void) {
	int i;
    for (i = 0; i < 300000; i++);
}

/**
 * @brief A delay function for button debouncing till it gets over.
 */
void Delay_Btn(void) {
    for (int i; i<500000/2; i++);
}

/** @} */

/**
 * @addtogroup GPIO_Public_Functions Public Functions
 * @{
 */

/**
 * @brief Initialize a GPIO pin with the specified configuration.
 *
 * @param GpioPinConfig GPIO_PinConfig_t structure containing pin configuration.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_InitPin(GPIO_PinConfig_t GpioPinConfig) {
    uint32_t temp = 0;
    uint8_t PortNum = GpioPinConfig.GPIO_PortIndex;
    uint8_t PinNum = GpioPinConfig.GPIO_PinNumber;
    uint8_t mode = GpioPinConfig.GPIO_PinMode;
    uint8_t PinSpeed = GpioPinConfig.GPIO_PinMode;
    uint8_t PinPuPdControl = GpioPinConfig.GPIO_PinPuPdControl;
    uint8_t PinPinOPType = GpioPinConfig.GPIO_PinPinOPType;

    Std_ReturnType ret = E_OK;
    if (PortNum >= Port_Indices) {
        ret = E_NOT_OK;
        return ret;
    }

    GPIO_PeripheralClockControl(PortNum, ENABLE);

    if (mode <= GPIO_MODE_ANALOG) {
        temp = (uint32_t)(mode << (2 * PinNum));
        pGPIO_Portx_RegisterControl[PortNum]->MODER &= ~(0x03 << (2U * PinNum));
        pGPIO_Portx_RegisterControl[PortNum]->MODER |= temp;

    } else {
        if (mode == GPIO_MODE_IT_FT) {
            pEXTI_RegisterControl[0]->FTSR |= (1 << PinNum);
            pEXTI_RegisterControl[0]->RTSR &= (uint32_t)~(1 << PinNum);

        } else if (mode == GPIO_MODE_IT_RT) {
            pEXTI_RegisterControl[0]->RTSR |= (1 << PinNum);
            pEXTI_RegisterControl[0]->FTSR &= (uint32_t)~(1 << PinNum);

        } else if (mode == GPIO_MODE_IT_RFT) {
            pEXTI_RegisterControl[0]->FTSR |= (1 << PinNum);
            pEXTI_RegisterControl[0]->RTSR |= (1 << PinNum);
        }

        uint32_t temp1 = PinNum / 4;
        uint16_t temp2 = (uint16_t)(PinNum % 4);
        uint16_t portcode = GPIO_BASEADDR_TO_CODE(pGPIO_Portx_RegisterControl[PortNum]);
        SYSCFG_PCLK_EN();
        pSYSCFG_RegisterControl[0]->EXTICR[temp1] = (uint32_t)portcode << (4 * temp2);
        pEXTI_RegisterControl[0]->IMR |= (1 << PinNum);
    }
    temp = 0;
    temp = ((uint32_t)PinSpeed << (2 * PinNum));
    pGPIO_Portx_RegisterControl[PortNum]->OSPEEDR &= (uint32_t)~(0x3 << (2 * PinNum));
    pGPIO_Portx_RegisterControl[PortNum]->OSPEEDR |= temp;

    temp = 0;
    temp = ((uint32_t)PinPuPdControl << (2 * PinNum));
    pGPIO_Portx_RegisterControl[PortNum]->PUPDR &= (uint32_t)~(0x3 << (2 * PinNum));
    pGPIO_Portx_RegisterControl[PortNum]->PUPDR |= temp;

    temp = 0;
    temp = ((uint32_t)PinPinOPType << (PinNum));
    pGPIO_Portx_RegisterControl[PortNum]->OTYPER &= (uint32_t)~(0x3 << (PinNum));
    pGPIO_Portx_RegisterControl[PortNum]->OTYPER |= temp;

    temp = 0;
    if (mode == GPIO_MODE_ALTFN) {
        uint32_t temp1, temp2;
        temp1 = PinNum / 8;
        temp2 = PinNum % 8;
        pGPIO_Portx_RegisterControl[PortNum]->AFR[temp1] &= (uint32_t)~(0xF << (4 * temp2));
        pGPIO_Portx_RegisterControl[PortNum]->AFR[temp1] |= ((uint32_t)GpioPinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
    return ret;
}

/**
 * @brief Deinitialize a GPIO port, resetting all registers to their default values.
 *
 * @param PortNum GPIO port index.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
/*
Std_ReturnType GPIO_DeInitPort(GPIO_PortNumIndexArr_t PortNum) {
    switch (PortNum) {
        case PortA:
            GPIOA_REG_RESET();
            break;
        case PortB:
            GPIOB_REG_RESET();
            break;
        case PortC:
            GPIOC_REG_RESET();
            break;
        case PortD:
            GPIOD_REG_RESET();
            break;
        case PortE:
            GPIOE_REG_RESET();
            break;
        case PortF:
            GPIOF_REG_RESET();
            break;
        case PortG:
            GPIOG_REG_RESET();
            break;
        case PortH:
            GPIOH_REG_RESET();
            break;
        case PortI:
            GPIOI_REG_RESET();
            break;
        default:
            break;
    }
    Std_ReturnType ret = E_OK;
    return ret;
}
*/

/**
 * @brief Read the input state of a GPIO pin.
 *
 * @param PortNum GPIO port index.
 * @param PinNum GPIO pin number.
 * @return uint8_t The input state of the pin (0 or 1).
 */
uint8_t GPIO_ReadInputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum) {
    uint8_t value;
    value = (uint8_t)((pGPIO_Portx_RegisterControl[PortNum]->IDR >> PinNum) & 0x00000001);
    return value;
}

/**
 * @brief Read the input state of a GPIO port.
 *
 * @param PortNum GPIO port index.
 * @param PinNum GPIO pin number.
 * @return uint16_t The input state of the port.
 */
uint16_t GPIO_ReadInputPort(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum) {
    uint16_t value;
    value = (uint16_t)(pGPIO_Portx_RegisterControl[PortNum]->IDR >> PinNum);
    return value;
}

/**
 * @brief Set or reset the output state of a GPIO pin.
 *
 * @param PortNum GPIO port index.
 * @param value GPIO_PIN_SET or GPIO_PIN_RESET.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_SetResetOutputPin(GPIO_PortNumIndexArr_t PortNum, uint8_t value) {
    Std_ReturnType ret = E_OK;
    switch (value) {
        case GPIO_PIN_SET:
            SET_REG(pGPIO_Portx_RegisterControl[PortNum]->ODR);
            break;
        case GPIO_PIN_RESET:
            CLR_REG(pGPIO_Portx_RegisterControl[PortNum]->ODR);
            break;
        default:
            ret = E_NOT_OK;
            break;
    }
    return ret;
}

/**
 * @brief Toggle the output state of a GPIO pin.
 *
 * @param PortNum GPIO port index.
 * @param PinNum GPIO pin number.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_ToggleOutputPin(GPIO_PortNumIndexArr_t PortNum, uint8_t PinNum) {
    Std_ReturnType ret = E_OK;
    if ((&pGPIO_Portx_RegisterControl[PortNum]) != NULL) {
        pGPIO_Portx_RegisterControl[PortNum]->ODR ^= (1 << PinNum);
    } else {
        ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Set or reset the output state of a GPIO pin.
 *
 * @param PortNum GPIO port index.
 * @param PinNum GPIO pin number.
 * @param value GPIO_PIN_SET or GPIO_PIN_RESET.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_WriteOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum, uint8_t value) {
    Std_ReturnType ret = E_OK;
    switch (value) {
        case GPIO_PIN_SET:
            pGPIO_Portx_RegisterControl[PortNum]->ODR |= (1 << PinNum);
            break;
        case GPIO_PIN_RESET:
            pGPIO_Portx_RegisterControl[PortNum]->ODR &= ~(1 << PinNum);
            break;
        default:
            ret = E_NOT_OK;
            break;
    }
    return ret;
}

/**
 * @brief Write a value to a GPIO port.
 *
 * @param PortNum GPIO port index.
 * @param value The value to write to the port.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_WriteOutputPort(GPIO_PortNumIndexArr_t PortNum, uint16_t value) {
    Std_ReturnType ret = E_OK;
    if ((&pGPIO_Portx_RegisterControl[PortNum]) != NULL) {
        pGPIO_Portx_RegisterControl[PortNum]->ODR = value;
    } else {
        ret = E_NOT_OK;
    }
    return ret;
}

/**
 * @brief Configure the interrupt for a specific GPIO pin.
 *
 * @param IRQNumber IRQ number.
 * @param EnorDi ENABLE or DISABLE macros.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (IRQNumber <= 31) {
            *NVIC_ISER0 = (1 << IRQNumber);
        } else if (IRQNumber >= 32 && IRQNumber < 64) {
            *NVIC_ISER1 = (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            *NVIC_ISER2 = (1 << (IRQNumber % 64));
        }
    } else if (EnorDi == DISABLE) {
        if (IRQNumber <= 31) {
            *NVIC_ICER0 = (1 << IRQNumber);
        } else if (IRQNumber >= 32 && IRQNumber < 64) {
            *NVIC_ICER1 = (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            *NVIC_ICER2 = (1 << (IRQNumber % 64));
        }
    }
    Std_ReturnType ret = E_OK;
    return ret;
}

/**
 * @brief Configure the priority of a specific IRQ.
 *
 * @param IRQNumber IRQ number.
 * @param IRQpriority Priority of the IRQ.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQpriority) {
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) = (unsigned int)IRQpriority << shift_amount;
    Std_ReturnType ret = E_OK;
    return ret;
}

/**
 * @brief Handle the interrupt for a specific GPIO pin.
 *
 * @param PinNum GPIO pin number.
 * @return Std_ReturnType E_OK if successful, E_NOT_OK otherwise.
 */
Std_ReturnType GPIO_IRQHandling(uint16_t PinNum) {
    Std_ReturnType ret = E_OK;
    if (pEXTI_RegisterControl[0]->PR & (1 << PinNum)) {

        pEXTI_RegisterControl[0]->PR |= (1 << PinNum);
    } else {

    	ret = E_NOT_OK;
    }
    return ret;
}



/** @} */

/*
 * GPIO.c
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#include "GPIO.h"

 /**************************************************************************************************************/
  /* creating an array of pointers to registers
   * the user (or the application layer) will not be allowed to have access to the hardware registers
   * because it's not his/her concern
   * he/she will only be allowed to call the interfacing functions in the GPIO.h file
   * for that reason a keyword  will be added
   *
   * A constant keyword is added because the location of the register never changes
   * so for safety no one can change the location of the pointer the only value that can be changed is the value INSIDE
   * the register ---> the value that the pointer is pointing to*/
 /**************************************************************************************************************/

/*
 * Note!! to not let high application files access the register by array of pointers use "static" keyword
 * Example: volatile RCC_AHB1ENR_Register *const RCC_Port_AHB1ClockEnable_Registers[Port_Indices]
 */

void delay(void){ // if delay is static, don't declare it in GPIO.h, and include GPIO.C in main.c
		int i;
		for (i=0; i<300000;i++);
	}

// if array of pointers not static, extern them in GPIO.h and include it in main.c,
// if it's static, do not extern it in GPIO.h and include GPIO.c in main.c
volatile GPIO_IDR_Register *const GPIO_IDRControl_Registers[Port_Indices]={
		GPIO_PortA_IDRControl,
        GPIO_PortB_IDRControl,
        GPIO_PortC_IDRControl,
        GPIO_PortD_IDRControl,
        GPIO_PortE_IDRControl,
        GPIO_PortF_IDRControl
 };

volatile GPIO_ODR_Register *const GPIO_ODRControl_Registers[Port_Indices]={
		GPIO_PortA_ODRControl,
        GPIO_PortB_ODRControl,
        GPIO_PortC_ODRControl,
        GPIO_PortD_ODRControl,
        GPIO_PortE_ODRControl,
        GPIO_PortF_ODRControl
 };

volatile GPIO_MODER_Register *const GPIO_ModeSelect_Registers[Port_Indices]={
		GPIO_PortA_ModeSelect,
        GPIO_PortB_ModeSelect,
        GPIO_PortC_ModeSelect,
        GPIO_PortD_ModeSelect,
        GPIO_PortE_ModeSelect,
        GPIO_PortF_ModeSelect
 };

volatile GPIO_AFRL_Register *const GPIO_AlternateFunctionLowSelect_Registers[Port_Indices]={

		GPIO_PortA_AlternateFunctionLowSelect,
		GPIO_PortB_AlternateFunctionLowSelect,
        GPIO_PortC_AlternateFunctionLowSelect,
        GPIO_PortD_AlternateFunctionLowSelect,
        GPIO_PortE_AlternateFunctionLowSelect,
        GPIO_PortF_AlternateFunctionLowSelect
 };

volatile GPIO_AFRH_Register *const GPIO_AlternateFunctionHighSelect_Registers[Port_Indices]={
		GPIO_PortA_AlternateFunctionHighSelect,
        GPIO_PortB_AlternateFunctionHighSelect,
        GPIO_PortC_AlternateFunctionHighSelect,
        GPIO_PortD_AlternateFunctionHighSelect,
        GPIO_PortE_AlternateFunctionHighSelect,
        GPIO_PortF_AlternateFunctionHighSelect
 };

volatile RCC_AHB1ENR_Register *const RCC_Port_AHB1ClockEnable_Registers[Port_Indices]={

		RCC_PortA_AHB1ClockEnable,
		RCC_PortB_AHB1ClockEnable,
		RCC_PortC_AHB1ClockEnable,
		RCC_PortD_AHB1ClockEnable,
		RCC_PortE_AHB1ClockEnable,
		RCC_PortF_AHB1ClockEnable
 };

volatile RCC_CR_Register *const RCC_Port_ClockControl_Register[Port_Indices]={
		RCC_ClockControl
};

volatile RCC_CFGR_Register *const RCC_Port_ClockConfig_Register[Port_Indices]={
		RCC_ClockConfig
};

GPIORegisters gpio_registers = {
    .GPIO_IDRControl_Registers = {
            GPIO_PortA_IDRControl,
            GPIO_PortB_IDRControl,
            GPIO_PortC_IDRControl,
            GPIO_PortD_IDRControl,
            GPIO_PortE_IDRControl,
            GPIO_PortF_IDRControl
    },
    .GPIO_ODRControl_Registers = {
    		GPIO_PortA_ODRControl,
            GPIO_PortB_ODRControl,
            GPIO_PortC_ODRControl,
            GPIO_PortD_ODRControl,
            GPIO_PortE_ODRControl,
            GPIO_PortF_ODRControl
    },
    .GPIO_ModeSelect_Registers = {
    		GPIO_PortA_ModeSelect,
            GPIO_PortB_ModeSelect,
            GPIO_PortC_ModeSelect,
            GPIO_PortD_ModeSelect,
            GPIO_PortE_ModeSelect,
            GPIO_PortF_ModeSelect
    },
    .GPIO_AlternateFunctionLowSelect_Registers = {
    		GPIO_PortA_AlternateFunctionLowSelect,
    		GPIO_PortB_AlternateFunctionLowSelect,
            GPIO_PortC_AlternateFunctionLowSelect,
            GPIO_PortD_AlternateFunctionLowSelect,
            GPIO_PortE_AlternateFunctionLowSelect,
            GPIO_PortF_AlternateFunctionLowSelect
    },
    .GPIO_AlternateFunctionHighSelect_Registers = {
    		GPIO_PortA_AlternateFunctionHighSelect,
            GPIO_PortB_AlternateFunctionHighSelect,
            GPIO_PortC_AlternateFunctionHighSelect,
            GPIO_PortD_AlternateFunctionHighSelect,
            GPIO_PortE_AlternateFunctionHighSelect,
            GPIO_PortF_AlternateFunctionHighSelect
    },

};

// Function to initialize GPIO pins
// void GPIO_Init(GPIORegisters *pGpioRegs, GPIO_PortNumIndexArr_t portIndex, GPIO_ConfigurePinNum_t pinNumber, uint8_t mode) {

Std_ReturnType GPIO_Init(GPIORegisters *pGpioRegs) {

    // Configure the mode of the GPIO pin
    uint32_t temp =0; // temp.register
    uint8_t Portindex = pGpioRegs->GPIO_PinConfig.GPIO_PortIndex;
    uint8_t PinNumber = pGpioRegs->GPIO_PinConfig.GPIO_PinNumber;
    uint8_t mode = pGpioRegs->GPIO_PinConfig.GPIO_PinMode;
    //uint8_t PinSpeed = pGpioRegs->GPIO_PinConfig.GPIO_PinMode;
    //uint8_t PinPuPdControl = pGpioRegs->GPIO_PinConfig.GPIO_PinPuPdControl;
    //uint8_t PinPinOPType = pGpioRegs->GPIO_PinConfig.GPIO_PinPinOPType;

    // Check if the portIndex is within a valid range
    Std_ReturnType ret = E_OK;
    if (Portindex >= Port_Indices) {
        // Handle error
    	ret = E_NOT_OK;
    	return ret;
    }

    temp =  (uint32_t) (mode << (2 * PinNumber));
    pGpioRegs->GPIO_ModeSelect_Registers[Portindex]->Register &= (uint32_t) ~(0x03 << (2U * PinNumber)); // Clear the bits
    pGpioRegs->GPIO_ModeSelect_Registers[Portindex]->Register |= (uint32_t)temp; // Set the mode

    /* Configure the output type of the GPIO pin
    pGpioRegs->GPIO_OTYPER_Registers[portIndex]->Register &= ~(1U << (PinNumber & 0x0F)); // Clear the bit
    pGpioRegs->GPIO_OTYPER_Registers[portIndex]->Register |= ((mode >> 4) << (PinNumber & 0x0F)); // Set the output type

    // Configure the output speed of the GPIO pin
    pGpioRegs->GPIO_OSPEEDR_Registers[portIndex]->Register &= ~(3U << (2U * PinNumber)); // Clear the bits
    pGpioRegs->GPIO_OSPEEDR_Registers[portIndex]->Register |= (speed << (2U * PinNumber)); // Set the speed

    // Configure the pull-up/pull-down of the GPIO pin
    pGpioRegs->GPIO_PUPDR_Registers[portIndex]->Register &= ~(3U << (2U * PinNumber)); // Clear the bits
    pGpioRegs->GPIO_PUPDR_Registers[portIndex]->Register |= (pupd << (2U * PinNumber)); // Set the pull-up/pull-down
	*/

    return ret;
}

/**************************************************************************************
 * GPIO APIs
 */

/**
 * @brief 
 * 
 * @param 
 * @param 
 */
uint8_t GPIO_ReadFromInputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum) // return the content of input data regiser
{
    //read the corresponding bit position of that pin in the input data register
    uint8_t value;

    //the IDR value shifted by the amount of PinNumber to get that bit position to LSB position of the IDR register
    value = (uint8_t) (( GPIO_IDRControl_Registers[PortNum]->Register >> PinNum) & 0x00000001 );

    return value; // 0 or 1
}

/**
 * @brief 
 * 
 * @param 
 * @param 
 */
uint16_t GPIO_ReadFromInputPort(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum)
{
    uint16_t value;

    value = (uint16_t) ( GPIO_IDRControl_Registers[PortNum]->Register >> PinNum);

    return value;
}

/**
 * @brief 
 * 
 * @param 
 * @param 
 */
Std_ReturnType GPIO_ToggleOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum)
{
    Std_ReturnType ret = E_OK;
    if((&GPIO_ODRControl_Registers[PortNum])!= NULL){
    	GPIO_ODRControl_Registers[PortNum]->Register ^= (1 << PinNum) ;
	}else{
		ret = E_NOT_OK;
	}
    return ret;
}


/**
 * @brief 
 * 
 * @param 
 * @param 
 */
Std_ReturnType GPIO_ReadFromOutputPin(GPIO_PortNumIndexArr_t PortNum, GPIO_ConfigurePinNum_t PinNum, uint8_t value ) // value = set 1 or reset 0
{
    Std_ReturnType ret = E_OK;
    switch(value){
        case GPIO_PIN_SET:
            GPIO_ODRControl_Registers[PortNum]->Register |= (1 << PinNum) ;
            break;
        case GPIO_PIN_RESET:
            GPIO_ODRControl_Registers[PortNum]->Register &= ~(1 << PinNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
} 

/**
 * @brief 
 * 
 * @param 
 * @param 
 */
Std_ReturnType GPIO_ReadFromOutputPort(GPIO_PortNumIndexArr_t PortNum, uint16_t value)
{
    Std_ReturnType ret = E_OK;
    if((&GPIO_ODRControl_Registers[PortNum])!= NULL){
        GPIO_ODRControl_Registers[PortNum]->Register = value ; // writing to the whole port to the given GPIO
    }else{
    	ret = E_NOT_OK;
    }
    return ret;
}


/**
 * @brief 
 * 
 * @param 
 * @param 
 */
Std_ReturnType GPIO_PeripheralClockControl (GPIO_PortNumIndexArr_t PortNum, uint8_t EnorDi)
{
    Std_ReturnType ret = E_OK;
    switch(EnorDi){
        case ENABLE:
            RCC_Port_AHB1ClockEnable_Registers[PortNum]->Register |= (1 << PortNum) ;
            break;
        case DISABLE:
            RCC_Port_AHB1ClockEnable_Registers[PortNum]->Register &= ~(1 << PortNum) ;
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}

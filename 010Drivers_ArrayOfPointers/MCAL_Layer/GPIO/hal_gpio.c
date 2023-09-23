/*
 * GPIO.c
 *
 *  Created on: Sep 16, 2023
 *      Author: mohamed
 */

#include "hal_gpio.h"

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

// if array of pointers not static, extern them in GPIO.h and include it in main.c,
// if it's static, do not extern it in GPIO.h and include GPIO.c in main.c

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

void delay(void){ // if delay is static, don't declare it in GPIO.h, and include GPIO.C in main.c
		int i;
		for (i=0; i<300000;i++);
	}

Std_ReturnType GPIO_ODRSetReset( GPIO_PortNumIndexArr_t PortNum, uint8_t value ) // value = set 1 or reset 0
{
    Std_ReturnType ret = E_OK;
    switch(value){
        case GPIO_PIN_SET:
        	SET_REG(pGPIO_Portx_RegisterControl[PortNum]->ODR);
            break;
        case GPIO_PIN_RESET:
        	CLR_REG(pGPIO_Portx_RegisterControl[PortNum]->ODR);
            break;
        default: ret = E_NOT_OK;
            break;
    }
    return ret;
}
// Function to initialize GPIO pins
// void GPIO_Init(GPIORegisters *pGpioRegs, GPIO_PortNumIndexArr_t portIndex, GPIO_ConfigurePinNum_t PinNum, uint8_t mode) {

Std_ReturnType GPIO_Init(GPIO_PinConfig_t GpioPinConfig) {

    // Configure the mode of the GPIO pin
    uint32_t temp = 0; // temp.register
    uint8_t PortNum = GpioPinConfig.GPIO_PortIndex;
    uint8_t PinNum = GpioPinConfig.GPIO_PinNumber;
    uint8_t mode = GpioPinConfig.GPIO_PinMode;
    uint8_t PinSpeed = GpioPinConfig.GPIO_PinMode;
    uint8_t PinPuPdControl = GpioPinConfig.GPIO_PinPuPdControl;
    uint8_t PinPinOPType = GpioPinConfig.GPIO_PinPinOPType;

    GPIO_PeripheralClockControl (PortNum, ENABLE);  // to avoid doing it in the code every time


    // Check if the portIndex is within a valid range
    Std_ReturnType ret = E_OK;
    if (PortNum >= Port_Indices) {
        // Handle error
    	ret = E_NOT_OK;
    	return ret;
    }
    if (mode <= GPIO_MODE_ANALOG)
    {
		temp =  (uint32_t) (mode << (2 * PinNum));
		pGPIO_Portx_RegisterControl[PortNum]->MODER &= ~(0x03 << (2U * PinNum)); // Clear the bits
		pGPIO_Portx_RegisterControl[PortNum]->MODER |= temp; // Set the mode
    }
    else
    {
        //the interrupt mode of detection falling or raising or both triggers
        if (mode == GPIO_MODE_IT_FT)
        {
            //1.configure the FTSR register
            pEXTI_RegisterControl[0]->FTSR |=  (1 << PinNum);
            //clear the corresponding RTSR bit
            pEXTI_RegisterControl[0]->RTSR &= (uint32_t) ~(1 << PinNum);

        } else if (mode == GPIO_MODE_IT_RT)
        {
            //2.configure the RSTR register
            pEXTI_RegisterControl[0]->RTSR |=  (1 << PinNum);
            //clear the corresponding FTSR bit
            pEXTI_RegisterControl[0]->FTSR &= (uint32_t) ~(1 << PinNum);

        }
        else if (mode == GPIO_MODE_IT_RFT)
        {
            //3.configure the FSTR and RSTR register
            pEXTI_RegisterControl[0]->FTSR |=  (1 << PinNum);
            pEXTI_RegisterControl[0]->RTSR |=  (1 << PinNum);
        }

        //2.configure the GPIO port selection from SYSCFG_EXTICR register (decide which GPIO port should take over this EXTI lines)
        uint32_t temp1 = PinNum / 4 ;
        uint16_t temp2 = (uint16_t) ( PinNum % 4 ) ;
        uint16_t portcode = GPIO_BASEADDR_TO_CODE( pGPIO_Portx_RegisterControl[PortNum] );
        SYSCFG_PCLK_EN();
        pSYSCFG_RegisterControl[0]->EXTICR[temp1] = (uint32_t) portcode << (4 * temp2) ;

        //3.enable the exti interrupt delivery from he peripheral to the processor using IMR
        pEXTI_RegisterControl[0]->IMR |=  (1 << PinNum);
    }

    temp = 0;
    //2. confiugure the speed
    temp = ( (uint32_t) PinSpeed << (2 * PinNum) );
    pGPIO_Portx_RegisterControl[PortNum]->OSPEEDR &= (uint32_t)~(0x3 << (2 * PinNum)) ; //clearing
    pGPIO_Portx_RegisterControl[PortNum]->OSPEEDR |= temp;

    temp = 0;
    //3. configure the pupd settigs
    temp = ( (uint32_t) PinPuPdControl << (2 * PinNum) );
    pGPIO_Portx_RegisterControl[PortNum]->PUPDR &= (uint32_t)~(0x3 << (2 * PinNum)) ; //clearing
    pGPIO_Portx_RegisterControl[PortNum]->PUPDR |= temp;

    temp = 0;
    //4. configure the optype
    temp = ( (uint32_t) PinPinOPType << ( PinNum) );
    pGPIO_Portx_RegisterControl[PortNum]->OTYPER &= (uint32_t)~(0x3 << ( PinNum )) ; //clearing
    pGPIO_Portx_RegisterControl[PortNum]->OTYPER |= temp;

    temp = 0;
    //5. configure alt functionality
    if (mode == GPIO_MODE_ALTFN) {

        uint32_t temp1, temp2 ;
        temp1 = PinNum /8 ; // temp1 = pinNumber(exp:9)/8 = 1
        temp2 = PinNum % 8;  // temp2 = pinNumber(exp:9)%8 = 1
        pGPIO_Portx_RegisterControl[PortNum]->AFR[temp1] &= (uint32_t)~(0xF << (4 * temp2)); //clearing the bit positions of the Pin
        // AFR[temp1=1] =  value << (4 * temp2)
        pGPIO_Portx_RegisterControl[PortNum]->AFR[temp1] |= ((uint32_t)GpioPinConfig.GPIO_PinAltFunMode << (4 * temp2));


    }

    return ret;
}

/**
 * @brief Deinitialize the GPIO port
 *
 * @param pGPIO_Portx_RegisterControl[PortNum] Pointer to GPIO port
 */
Std_ReturnType GPIO_DeInit (GPIO_PortNumIndexArr_t PortNum)
{

    if (pGPIO_Portx_RegisterControl[PortNum] == GPIOA)
    {
        GPIOA_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOB)
    {
        GPIOB_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOC)
    {
        GPIOC_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOD)
    {
        GPIOD_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOE)
    {
        GPIOE_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOF)
    {
        GPIOF_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOG)
    {
        GPIOG_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOH)
    {
        GPIOH_REG_RESET();

    } else if(pGPIO_Portx_RegisterControl[PortNum] == GPIOI)
    {
        GPIOI_REG_RESET();

    }

    Std_ReturnType ret = E_OK;

    return ret;

} // RCC AHB1 peripheral reset register(p233 RF_M) will make all the reg of GPIO to reset value in one shot


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

    //the IDR value shifted by the amount of PinNum to get that bit position to LSB position of the IDR register
    value = (uint8_t) (( pGPIO_Portx_RegisterControl[PortNum]->IDR >> PinNum) & 0x00000001 );

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

    value = (uint16_t) ( pGPIO_Portx_RegisterControl[PortNum]->IDR >> PinNum);

    return value;
}

/**
 * @brief 
 * 
 * @param 
 * @param 
 */
Std_ReturnType GPIO_ToggleOutputPin(GPIO_PortNumIndexArr_t PortNum, uint8_t PinNum)
{
    Std_ReturnType ret = E_OK;
    if((&pGPIO_Portx_RegisterControl[PortNum])!= NULL){
    	pGPIO_Portx_RegisterControl[PortNum]->ODR ^= (1 << PinNum) ;
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
        	pGPIO_Portx_RegisterControl[PortNum]->ODR |= (1 << PinNum) ;
            break;
        case GPIO_PIN_RESET:
        	pGPIO_Portx_RegisterControl[PortNum]->ODR &= ~(1 << PinNum) ;
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
    if((&pGPIO_Portx_RegisterControl[PortNum])!= NULL){
    	pGPIO_Portx_RegisterControl[PortNum]->ODR = value ; // writing to the whole port to the given GPIO
    }else{
    	ret = E_NOT_OK;
    }
    return ret;
}

/****************************************************************************************
 *
 */

/************************************************************************************************************************
 * IRQ configuration and ISR handling
 */

//all this configuration is processor specific or side

/**
 * @brief Configure the interrupt for a specific GPIO pin
 *
 * @param IRQNumber IRQ number
 * @param EnorDi ENABLE or DISABLE macros
 */
Std_ReturnType GPIO_IRQinterruptConfig (uint8_t IRQNumber, uint8_t EnorDi )  // to configure the IRQ number of GPIO pin enable/setting up priority...etc
{
    // in cortex M4 generic user guide p219 in NVIC registers
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <=31 )
        {
            //program ISER0 register
            //derefrence the address of The ISER0 register and put the value
            *NVIC_ISER0 = (1 << IRQNumber);
        }
        else if (IRQNumber >=32 && IRQNumber < 64 )
        {
            //program ISER1 register
            *NVIC_ISER1 = (1 << IRQNumber %32); // 32 is the 0 so 32%32=0

        } else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            //program ISER2 register // 64 to 95
            *NVIC_ISER2 = (1 << IRQNumber %64);

        }
    }
    else if(EnorDi == DISABLE) {

        if (IRQNumber <=31 )
        {
            //program ICER0 register
            *NVIC_ICER0 = (1 << IRQNumber);
        }
        else if (IRQNumber >=32 && IRQNumber < 64 )
        {
            //program ICER1 register
            *NVIC_ICER1 = (1 << IRQNumber %32 );

        } else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            //program ICER2 register // 64 to 95
            *NVIC_ICER2 = (1 << IRQNumber %64);

        }

    }

    Std_ReturnType ret = E_OK;

    return ret;

}

/**
 * @brief Configure the priority of a specific IRQ
 *
 * @param IRQNumber IRQ number
 * @param IRQpriority Priority of the IRQ
 */
Std_ReturnType GPIO_IRQperiorityConfig(uint8_t IRQNumber,uint32_t IRQpriority)
{
    // there is 60 interrupt priority register IPR each 32Bit 4section eachsection 8bits
    //for exp : from the IRQ237 -> 237/4 = 59( touch IPR59 register ) ;
    // 237%4 = 1(the section position of the IRQ237) ;  shift by 1*8 to touch IRQ237

    // 1.first let's find out the ipr register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;
    *(NVIC_PR_BASE_ADDR + iprx) = (unsigned int)IRQpriority << shift_amount ; // uint32_t will increment by 4 each increment

    Std_ReturnType ret = E_OK;

    return ret;
}

/**
 * @brief Handle the interrupt for a specific GPIO pin
 *
 * @param PinNum GPIO pin number
 */
Std_ReturnType GPIO_IRQHandling (uint16_t PinNum)
{
    // clear the exti pr register corresponding to the pin number
    if ( pEXTI_RegisterControl[0]->PR & (1 << PinNum)) // if the value is set then the interrupt is pended
    {
        //clear that pending register bit with 1
        pEXTI_RegisterControl[0]->PR |= (1 << PinNum);
    }
    Std_ReturnType ret = E_OK;

    return ret;  //whenever interrupt occurs the userApp call this fonc to process that interrupt
}

/**
 * @file hal_i2c.c
 * @author your name (you@domain.com)
 * @brief This file contains the implementation of the I2C driver APIs.
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

// Including necessary header file
#include "hal_i2c.h" // the I2C driver headerfile

volatile I2C_RegDef_t static *const I2Cx_RegisterControl[3] = {
			I2C1_RegisterControl,
			I2C2_RegisterControl,
			I2C3_RegisterControl
};

// Array of prescaler values for AHB and APB1 buses
static uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
static uint16_t APB1_PreScaler[4] = {2,4,8,16};

// Helper functions (static/private)
static void I2C_GenerateStartCondition(I2C_Handle_t *pI2CHandle);
static void I2C_ExcuteAddressPhaseWrite(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr);
static void I2C_ExcuteAddressPhaseRead(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandlingTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandlingRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/**
 * @brief Generates the start condition on the I2C bus.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral register structure.
 */
static void I2C_GenerateStartCondition ( I2C_Handle_t *pI2CHandle)
{
    //to generate the start or the repeated one
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 |= (1<<I2C_CR1_START);
}

/**
 * @brief Generates the stop condition on the I2C bus.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral register structure.
 */
void I2C_GenerateStopCondition (I2C_Handle_t *pI2CHandle)
{
    //to generate the stop condition after sending the byte end
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 |= (1<<I2C_CR1_STOP);
}

/**
 * @brief Executes the address phase for write operation on the I2C bus.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral register structure.
 * @param SlaveAddr Address of the slave device.
 */
static void I2C_ExcuteAddressPhaseWrite( I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr )
{
    SlaveAddr = (uint8_t)( SlaveAddr << 1)  ; // to make space for r/nw bit
    SlaveAddr &= ~(1);                        // SlaveAddr is SlaveAddr + r/nw bit , and clear the 0th bit (write bit)
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR = SlaveAddr;
}

/**
 * @brief Executes the address phase for read operation on the I2C bus.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral register structure.
 * @param SlaveAddr Address of the slave device.
 */
static void I2C_ExcuteAddressPhaseRead( I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr )
{
    SlaveAddr = (uint8_t)( SlaveAddr << 1)  ; // to make space for r/nw bit
    SlaveAddr |= (1);                        // SlaveAddr is SlaveAddr + r/nw bit , and set the 0th bit (read bit)
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR = SlaveAddr;
}

/**
 * @brief Clears the address flag and handles the appropriate actions based on the I2C mode.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
static void I2C_ClearAddrFlag( I2C_Handle_t *pI2CHandle )
{
    uint32_t dummy_Read ;
    // check the device mode
    if( I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2 & (1 << I2C_SR2_MSL)) {

        // device in master mode
        if ( pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

            if ( pI2CHandle->RxSize == 1 ) {

                // first disable ack
                I2C_ManageAcking (pI2CHandle, I2C_ACK_DISABLE );

                // clear the ADDR flag ( read SR1 , read SR2 )
                dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1;
                dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2;
                (void)dummy_Read;

            }
            /*else if ( pI2CHandle->RxSize > 1 ) {

            // clear the ADDR flag ( read SR1 , read SR2 )
            dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1;
            dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2;
            (void)dummy_Read;

            }*/
        }
        else {
            // clear the ADDR flag ( read SR1 , read SR2 )
            dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1;
            dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2;
            (void)dummy_Read;
        }
    }
    else {
        //device in slave mode

        dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1;
        dummy_Read = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2;
        (void)dummy_Read;

    }
}


/*
 * Peripheral clock setup
 */

/**
 * @brief Enables or disables the peripheral clock for the given I2C peripheral.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral register structure.
 * @param EnorDi Enable or disable action. Use ENABLE to enable and DISABLE to disable.
 */
void I2C_PeriClockControl( I2C_Handle_t *pI2CHandle, uint8_t EnorDi )
{
    if (EnorDi == ENABLE)
    {
        if (I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C1)
        {
            I2C1_PCLK_EN();

        } else if(I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C2)
        {
            I2C2_PCLK_EN();

        } else if(I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if (EnorDi == DISABLE)
        {
            if (I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C1)
            {
                I2C1_PCLK_DI();

            } else if(I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C2)
            {
                I2C2_PCLK_DI();

            } else if(I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C3)
            {
                I2C3_PCLK_DI();
            }
        }
    }
}

/**
 * @brief Retrieves the output clock frequency of the PLL.
 *
 * @return uint32_t The output clock frequency of the PLL.
 */
uint32_t RCC_GETPLLOutputClock(void)
{
    uint32_t PLLclk = 0; // not used in the MCU1 course
    return PLLclk;
}

/**
 * @brief Retrieves the PCLK1 value, which is the frequency of the APB1 bus.
 *
 * @return uint32_t The PCLK1 value.
 */
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, systemClk = 0 ;
    uint8_t clksrc, temp, ahbp, apb1p ;
    clksrc = ((RCC->CFGR >> 2) & 0x03 );

    if (clksrc == 0)
    {
        systemClk = 16000000 ;

    } else if (clksrc == 1) {

        systemClk = 8000000;

    } else if (clksrc == 2)
    {
        systemClk = RCC_GETPLLOutputClock() ;

    }

    // for the AHB1
    temp = ((RCC->CFGR >> 4) & 0xF ) ;

    if (temp < 8)
    {
        ahbp = 1 ;

    } else {
        ahbp = (uint8_t)AHB_PreScaler[temp-8];
    }

    // for the APB1
    temp = ((RCC->CFGR >> 10) & 0x7 ) ;

    if (temp < 4)
    {
        apb1p = 1 ;

    } else {
        apb1p = (uint8_t)APB1_PreScaler[temp-4];
    }

    pclk1 = (systemClk/ahbp)/apb1p ;

    return pclk1;

}

/*************************************************************************************************************************
 * Init and DeInit
 */

/**
 * @brief Initializes the I2C peripheral with the provided configuration settings.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_Init( I2C_Handle_t *pI2CHandle )
{
    // enalbling the peripheral clock  (internally )
    I2C_PeriClockControl (pI2CHandle, ENABLE );  // to avoid doing it in the code every time

    /*
    steps to follow in " any MCU " when the I2C peripheral is disabled:
    - configure the mode (standard or fast)
    - configure the speed of the serial clock (SCL) Rq : when we use high freq we can't drive I2C to longer distances
    - configure the device address ( applicable when device is slave )
    - Enable the Acking
    - configure the rise time ( slew rate ) for I2C pins (SDA&SCL)
     */

    // Ack control bit
    uint32_t tempreg = 0;
    tempreg |= (uint32_t)( pI2CHandle->I2C_Config.I2C_ACKControl << 10 );
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 = tempreg  ;

    // configure the Freq field of CR2
    tempreg = 0;
    tempreg |= RCC_GetPCLK1Value() / 1000000U ; // need only 16 from the 16 MHZ
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 = ( tempreg & 0x3F ) ;

    tempreg = 0;
    // program the device own address
    tempreg |= (uint32_t) (pI2CHandle->I2C_Config.I2C_DeviceAdress<<1) ;
    tempreg |= (1<<14); // the RM tells us to keep the 14th bit as 1
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->OAR1 = tempreg;

    // CCR calculations for SCLK speed
    uint16_t ccr_value ;
    tempreg = 0 ;

    if (pI2CHandle->I2C_Config.I2C_SCLspeed <= I2C_SC_SPEED_SM )
    {
        //mode is standard mode
        // we have: Thigh+Tlow= TscL = 2*CCR*Tpclk => CCR = TscL /(2*Tpclk) = Fpclk /(2*FscL) ; (FscL) = giving by the user
        ccr_value = (uint16_t) ( RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLspeed )) ;

        tempreg |= (ccr_value & 0xFFF);
    } else {

        // mode is fast mode
        tempreg |= (1<< I2C_CCR_FS);
        tempreg |= (uint32_t)( pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if ( pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2 )
        {
            // if DUTY = 0 : Thigh+Tlow= TscL = 3*CCR*Tpclk =>  CCR = TscL /(3*Tpclk) = Fpclk /(3*FscL)
            ccr_value = (uint16_t) ( RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLspeed )) ;
        }

        else {
            // if DUTY = 1 : Thigh+Tlow= TscL = 25*CCR*Tpclk => CCR = TscL /(25*Tpclk) = Fpclk /(25*FscL)
            ccr_value = (uint16_t) ( RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLspeed )) ;

        }
        tempreg |= (ccr_value & 0xFFF);
    }
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CCR = tempreg;

    tempreg = 0;
    // TRISE configuration
    if (pI2CHandle->I2C_Config.I2C_SCLspeed <= I2C_SC_SPEED_SM )
    {

        //mode is standard mode
        tempreg = (RCC_GetPCLK1Value()/ 1000000U ) + 1 ;

    } else {

        // mode is fast mode
        tempreg = (( RCC_GetPCLK1Value() * 300 ) / 1000000000U ) + 1 ;

    }

    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->TRISE = (tempreg & 0x3F) ;

}

/**
 * @brief Deinitializes the I2C peripheral.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral register structure.
 */
void I2C_DeInit (I2C_Handle_t *pI2CHandle)
{

    if (I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C1)
    {
        I2C1_REG_RESET();

    } else if(I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C2)
    {
        I2C2_REG_RESET();

    } else if(I2Cx_RegisterControl[pI2CHandle->I2cIndex] == I2C3)
    {
        I2C3_REG_RESET();
    }
}

/************************************************************************************************************************
 * Data send and receive
 */

/**
 * @brief Sends data over I2C as a master to the specified slave device.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pTxBuffer Pointer to the transmit buffer.
 * @param Len Number of bytes to transmit.
 * @param SlaveAddr Address of the slave device.
 * @param SR Generate Stop Condition after the transaction (ENABLE or DISABLE).
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t SR)
{
    //1. generate the start condition
    I2C_GenerateStartCondition (pI2CHandle);

    //2. confirm that start generation is completed by checking the SB flag in the SR1
    //   Note: until SB is cleared SCL will be stretched (pulled to Low)
    while( ! I2C_GetFlagStatus( pI2CHandle, I2C_FLAG_SB )  );

    //3. Send the address of the slave with r/nw bit set to w(0) (total 8bit)
    I2C_ExcuteAddressPhaseWrite(pI2CHandle, SlaveAddr);

    //4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
    while( ! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_ADDR )  );

    //5. clear the ADDR flag according to its software sequence
    //   Note : until ADDR is cleared SCL will be stretched (pulled to Low)
    I2C_ClearAddrFlag( pI2CHandle );

    //6. send the data until Len becomes 0
    while(Len > 0)
    {
        // wait till TXE is set which mean that the DR is empty
        while( ! I2C_GetFlagStatus(pI2CHandle,I2C_FLAG_TxE ) );
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    //7. when Len becomes 0 wait for TXE=1 and BTF=1 before generating the stop condition
    //   Note : TXE=1 ,BTF=1 , means that both DR and SR are empty and next transmission should begin
    //   when BTF=1 SCL will be stretched (pulled to Low)

    while( ! I2C_GetFlagStatus(pI2CHandle,I2C_FLAG_TxE ) );

    while( ! I2C_GetFlagStatus(pI2CHandle,I2C_FLAG_BTF ) )
        ; // use {} instead of ; or put ; on a separate line to silence this warning


    //8. Generate the stop condition and master need not to wait for the compilation of stop condition
    //   Note : generating STOP, automatically clear the BTF

    if ( SR == I2C_DISABLE_Sr ) { // it is preffered to use the repeated start than using a stop condition after each transaction to avoid giving the chance to another master to claim the bus
        I2C_GenerateStopCondition ( pI2CHandle);
    }

}

/**
 * @brief Receives data over I2C as a master from the specified slave device.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pRxBuffer Pointer to the receive buffer.
 * @param Len Number of bytes to receive.
 * @param SlaveAddr Address of the slave device.
 * @param SR Generate Stop Condition after the transaction (ENABLE or DISABLE).
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t SR)
{
    //1. generate the start condition
    I2C_GenerateStartCondition (pI2CHandle);

    //2. confirm that start generation is completed by checking the SB flag in the SR1
    //   Note: until SB is cleared SCL will be stretched (pulled to Low)
    while( ! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_SB )  );

    //3. Send the address of the slave with r/nw bit set to R(1) (total 8bit)
    I2C_ExcuteAddressPhaseRead(pI2CHandle, SlaveAddr);

    //4. Confirm that the address phase is completed by checking the ADDR flag in the SR1
    while( ! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_ADDR ) );

    //5. procedure to read only 1 byte from slave
    if (Len == 1) // you'll find the steps in RM P_850
    {
        // disable acking
        I2C_ManageAcking(pI2CHandle, I2C_ACK_DISABLE );

        // clear the ADDR flag according to its software sequence
        // Note : until ADDR is cleared SCL will be stretched (pulled to Low)
        I2C_ClearAddrFlag( pI2CHandle );

        // wait until RXNE becomes 1
        while( ! I2C_GetFlagStatus(pI2CHandle,I2C_FLAG_RxNE ) );

        // generate the stop condition
        if ( SR == I2C_DISABLE_Sr ) {
            I2C_GenerateStopCondition ( pI2CHandle );
        }

        // read data in to buffer
        *pRxBuffer = (uint8_t) I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR ;

    }

    //6. procedure to read data from the slave when Len > 1
    if (Len > 1)
    {
        // clear the ADDR flag according to its software sequence
        // Note : until ADDR is cleared SCL will be stretched (pulled to Low)
        I2C_ClearAddrFlag( pI2CHandle );

        // read data unitl it Len becomes 0
        for(uint32_t i = Len ; i > 0 ; i--)
        {
            // wait till RXNE is set
            while( ! I2C_GetFlagStatus(pI2CHandle,I2C_FLAG_RxNE ) );

            if (i == 2) // if last 2 bytes are remaining
            {
                // clear the ack bit
                I2C_ManageAcking(pI2CHandle, I2C_ACK_DISABLE );

                // generate the stop condition
                if ( SR == I2C_DISABLE_Sr ) {
                    I2C_GenerateStopCondition (pI2CHandle);
                }
            }
            // read the data from the data register in to buffer
            *pRxBuffer = (uint8_t) I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR ;
            // incerement the buffer address
            pRxBuffer++;
        }
    }
    // re-enable acking
    if ( pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {

        I2C_ManageAcking(pI2CHandle, I2C_ACK_ENABLE );
    }
}



/************************************************************************************************************************
 * IRQ configuration and ISR handling
 */

//all this configuration is processor specific or side
/**
 * @brief Configures the interrupt for the specified IRQ number.
 *
 * @param IRQNumber IRQ number to be configured.
 * @param EnorDi Enable or disable the IRQ (ENABLE or DISABLE).
 */
void I2C_IRQInterruptConfig( uint8_t IRQNumber, uint8_t EnorDi )  // to configure the IRQ number of GPIO pin enable/setting up priority...etc
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
}

/**
 * @brief Configures the priority for the specified IRQ number.
 *
 * @param IRQNumber IRQ number to be configured.
 * @param IRQPriority Priority to be set for the IRQ.
 */
void I2C_IRQPriorityConfig( uint8_t IRQNumber, uint32_t IRQPriority )
{
    // there is 60 interrupt priority register IPR each 32Bit 4section eachsection 8bits
    //for exp : from the IRQ237 -> 237/4 = 59( touch IPR59 register ) ;
    // 237%4 = 1(the section position of the IRQ237) ;  shift by 1*8 to touch IRQ237

    // 1.first let's find out the ipr register
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;
    *(NVIC_PR_BASE_ADDR + iprx) = (unsigned int)IRQPriority << shift_amount ; // uint32_t will increment by 4 each increment

}

/**
 * @brief Sends data over I2C as a master using interrupt-driven communication.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pTxBuffer Pointer to the transmit buffer.
 * @param Len Number of bytes to transmit.
 * @param SlaveAddr Address of the slave device.
 * @param Sr Generate Repeated Start condition (ENABLE or DISABLE).
 * @return uint8_t Current state of the I2C peripheral (I2C_BUSY_IN_TX or I2C_BUSY_IN_RX).
 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        // saving all the details into the header
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Implement code to Generate START Condition
        I2C_GenerateStartCondition (pI2CHandle); // after this SB will be set and an interrupt will be triggred on The IRQn and sending the Addr..etc will be taking care by the ISR

        //Implement the code to enable ITBUFEN Control Bit
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

        //Implement the code to enable ITEVTEN Control Bit
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

        //Implement the code to enable ITERREN Control Bit
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= ( 1 << I2C_CR2_ITERREN);

    }

    return busystate;

}

/**
 * @brief Sends data over I2C as a master using interrupt-driven communication.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 * @param pRxBuffer Pointer to the receive buffer.
 * @param Len Number of bytes to receive.
 * @param SlaveAddr Address of the slave device.
 * @param Sr Generate Repeated Start condition (ENABLE or DISABLE).
 * @return uint8_t Current state of the I2C peripheral (I2C_BUSY_IN_TX or I2C_BUSY_IN_RX).
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        //Implement code to Generate START Condition
        I2C_GenerateStartCondition (pI2CHandle);

        //Implement the code to enable ITBUFEN Control Bit
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

        //Implement the code to enable ITEVTEN Control Bit
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

        //Implement the code to enable ITERREN Control Bit
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= ( 1 << I2C_CR2_ITERREN);

    }

    return busystate;
}

/**
 * @brief Handles the TXE (Transmit Empty) interrupt.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
static void I2C_MasterHandlingTXEInterrupt(I2C_Handle_t *pI2CHandle )
{
    if (pI2CHandle->TxLen > 0) {

        //1. load the data in to DR
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR = (uint32_t)*(pI2CHandle->pTxBuffer);
        //2. decrement the TxLen
        pI2CHandle->TxLen--;
        //3. Increment the buffer address
        pI2CHandle->pTxBuffer++;

    }

}

/**
 * @brief Handles the RXNE (Receive Not Empty) interrupt.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
static void I2C_MasterHandlingRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
    if (pI2CHandle->RxSize == 1) {
        // read DR
        *(pI2CHandle->pRxBuffer) = (uint8_t) I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR ;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxSize > 1) {

        if(pI2CHandle->RxLen == 2) {
            // clear the ack bit
            I2C_ManageAcking(pI2CHandle, I2C_ACK_DISABLE );
        }

        // read DR
        *(pI2CHandle->pRxBuffer) = (uint8_t) I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR ;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if ( pI2CHandle->RxLen==0) {

        // close the I2C data reception and notify the application

        //1. generate the stop condition
        if ( pI2CHandle->Sr == I2C_DISABLE_Sr ) {
            I2C_GenerateStopCondition (pI2CHandle);
        }

        //2. close the I2C Rx
        I2C_CloseReceiveData(pI2CHandle);

        //3. notify the application
        I2C_ApplicationEventCallback( pI2CHandle, I2C_EV_RX_CMPLT );

    }
}

/**
 * @brief Handles I2C event interrupts and calls appropriate event callback functions.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_EV_IRQHandling( I2C_Handle_t *pI2CHandle)
{
    // Interrupt handling API is common for both master and slave mode of a device

    uint32_t temp1, temp2, temp3 ;
    temp1 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 & (1 << I2C_CR2_ITEVTEN) ;
    temp2 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 & (1 << I2C_CR2_ITBUFEN) ;


    temp3 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & (1 << I2C_SR1_SB) ;
    //1. Handle for interrupt generated by SB event
    //Note : SB flag is only applicable in master mode

    if ( temp1 && temp3)
    {
        // the interrupt is generated because of SB event
        // this block will not be executed in slave mode because for slave SB is always zero
        // in this block lets execute the address phase
        if ( pI2CHandle->TxRxState == I2C_BUSY_IN_TX )
        {
            I2C_ExcuteAddressPhaseWrite(pI2CHandle,pI2CHandle->DevAddr);

        } else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX ) {

            I2C_ExcuteAddressPhaseRead(pI2CHandle,pI2CHandle->DevAddr);
        }
    }

    temp3 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & (1 << I2C_SR1_ADDR) ;
    //2. Handle for interrupt generated by ADDR event
    //Note : when master mode : Address is sent
    //       when slave mode  : Address matched with own address
    if ( temp1 && temp3)
    {
        // the interrupt is generated because of ADDR event
        I2C_ClearAddrFlag(pI2CHandle);

    }

    temp3 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & (1 << I2C_SR1_BTF) ;
    //3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
    if ( temp1 && temp3)
    {
        // the interrupt is generated because of BTF event
        //check the I2C application state
        if ( pI2CHandle->TxRxState == I2C_BUSY_IN_TX ) {

            // make sure that TXE is also set
            if( I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & (1 << I2C_SR1_TxE)) {

                // now we are sure that : BTF, TXE = 1
                if(pI2CHandle->TxLen == 0 ) {

                    //1. generate the stop condition
                    if (pI2CHandle->Sr == I2C_DISABLE_Sr ) {

                        I2C_GenerateStopCondition (pI2CHandle);
                    }
                    //2. Reset all the memeber element of the handle structure (closing the I2C data transmission)
                    I2C_CloseSendData(pI2CHandle) ;

                    //3. notify the application about transmission complete
                    I2C_ApplicationEventCallback( pI2CHandle, I2C_EV_TX_CMPLT );
                }
            }

        } else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX ) {

            // if it's busy in RX we have nothing to do here
            // we don't close the I2C reception like the Tx one
            // ; in eclipse maybe no warning from this unlike keil
        }
    }

    temp3 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & (1 << I2C_SR1_STOPF) ; // this is checking STOPF and reading the SR1 at the same time
    //4. Handle for interrupt generated by STOPF event
    //Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
    // The below code block will not be executed by the master since STOPF will not set in master mode
    if ( temp1 && temp3 )
    {
        // the interrupt is generated because of STOPF event
        // to clear the STOPF :  1)read the SR1(done)  2)write the CR1 )

        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 |= 0x0000; // this will not affect the CR1

        //3. notify the application that stop is detected
        I2C_ApplicationEventCallback( pI2CHandle, I2C_EV_STOP );
    }

    temp3 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & (1 << I2C_SR1_TxE) ;
    //5. Handle for interrupt generated by TXE event
    if ( temp1 && temp2 && temp3 )
    {
        // to check if the device behave in the master mode
        if( I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2 & (1 << I2C_SR2_MSL))
        {
            // TXE is set
            // the interrupt is generated because of TXE event

            // we have to do data transmission
            if ( pI2CHandle->TxRxState == I2C_BUSY_IN_TX ) {

                I2C_MasterHandlingTXEInterrupt(pI2CHandle);

            }
        }
        else {

            // for slave mode

            // make sure that the slave is really in transmitter mode (if TRA is set)
            if ( ! ( I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2 & (1 << I2C_SR2_TRA ) ) ) {

                //( slave receive data from master )
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }

    temp3 = I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & (1 << I2C_SR1_RxNE) ;
    //6. Handle for interrupt generated by RXNE event
    if ( temp1 && temp2 && temp3)
    {
        // to check if the device behave in the master mode
        if( I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2 & (1 << I2C_SR2_MSL))
        {
            // RXNE flag is set
            // the interrupt is generated because of RXNE event
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

                I2C_MasterHandlingRXNEInterrupt(pI2CHandle);


            }
        }
        else {

            // for slave mode

            // make sure that the slave is really in Receiver mode (if TRA is clear )
            if ( I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR2 &= (1 << I2C_SR2_TRA )) {

                //( slave send data to master )
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }

        }
    }
}

/**
 * @brief Closes the I2C data transmission, disables relevant interrupts, and resets handle members.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    // implement the code to disabe ITBUFEN Control Bit
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 &= ~(unsigned int) (1 << I2C_CR2_ITBUFEN ) ;
    // implement the code to disabe ITEVTEN Control Bit
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 &= ~(unsigned int) (1 << I2C_CR2_ITEVTEN ) ;

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;


}

/**
 * @brief Closes the I2C data reception in slave mode, disables relevant interrupts, and resets handle members.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    // implement the code to disabe ITBUFEN Control Bit
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 &= ~(unsigned int) (1 << I2C_CR2_ITBUFEN ) ;
    // implement the code to disabe ITEVTEN Control Bit
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 &= ~(unsigned int) (1 << I2C_CR2_ITEVTEN ) ;

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0; //Rxsize is used in the ISR code to manage the data reception

    // re-enable acking only if the ACK control is set to true
    if ( pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {

        I2C_ManageAcking(pI2CHandle, I2C_ACK_ENABLE );
    }

}

/**
 * @brief Sends data in slave mode.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral.
 * @param data Data to be sent.
 */
void I2C_SlaveSendData( I2C_Handle_t *pI2CHandle, uint8_t data )
{
    I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR = data;
}

/**
 * @brief Receives data in slave mode.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral.
 * @return uint8_t Received data.
 */
uint8_t I2C_SlaveReceiveData( I2C_Handle_t *pI2CHandle)
{
    return (uint8_t) I2Cx_RegisterControl[pI2CHandle->I2cIndex]->DR;
}

/**
 * @brief Handles I2C error interrupts and calls appropriate error callback functions.
 *
 * @param pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

    uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
    temp2 = (I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2) & ( 1 << I2C_CR2_ITERREN);


    /***********************Check for Bus error************************************/
    temp1 = (I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1) & ( 1<< I2C_SR1_BERR);
    if(temp1  && temp2 )
    {
        //This is Bus error

        //Implement the code to clear the buss error flag
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 &= ~(unsigned int)( 1 << I2C_SR1_BERR);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
    }

    /***********************Check for arbitration lost error************************************/
    temp1 = (I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1) & ( 1 << I2C_SR1_ARLO );
    if(temp1  && temp2)
    {
        //This is arbitration lost error

        //Implement the code to clear the arbitration lost error flag
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 &= ~(unsigned int)( 1 << I2C_SR1_ARLO);


        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

    }

    /***********************Check for ACK failure  error************************************/

    temp1 = (I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1) & ( 1 << I2C_SR1_AF);
    if(temp1  && temp2)
    {
        //This is ACK failure error

        //Implement the code to clear the ACK failure error flag
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 &= ~(unsigned int)( 1 << I2C_SR1_AF);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);

    }

    /***********************Check for Overrun/underrun error************************************/
    temp1 = (I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1) & ( 1 << I2C_SR1_OVR);
    if(temp1  && temp2)
    {
        //This is Overrun/underrun

        //Implement the code to clear the Overrun/underrun error flag
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 &= ~(unsigned int)( 1 << I2C_SR1_OVR);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);

    }

    /***********************Check for Time out error************************************/
    temp1 = (I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1) & ( 1 << I2C_SR1_TIMEOUT);
    if(temp1  && temp2)
    {
        //This is Time out error

        //Implement the code to clear the Time out error flag
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 &= ~(unsigned int)( 1 << I2C_SR1_TIMEOUT);

        //Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);

    }

}


/************************************************************************************************************************
 * other Peripheral control APIs
 */

/**
 * @brief Enables or disables the I2C peripheral.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral.
 * @param EnorDi ENABLE to enable, DISABLE to disable.
 */
void I2C_PeripheralControl( I2C_Handle_t *pI2CHandle, uint8_t EnorDi )
{
    if (EnorDi == ENABLE)
    {
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 |= (1 << I2C_CR1_PE ) ;
    }

    else
    {
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 &= ~(unsigned int)(1 << I2C_CR1_PE ) ;

    }

}

/**
 * @brief Gets the status of a specific flag in the I2C status register.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral.
 * @param FlagName The flag to check status for.
 * @return uint8_t FLAG_SET if the flag is set, FLAG_RESET if not set.
 */
uint8_t I2C_GetFlagStatus( I2C_Handle_t *pI2CHandle, uint8_t FlagName )
{

    if ( I2Cx_RegisterControl[pI2CHandle->I2cIndex]->SR1 & FlagName )
    {
        return FLAG_SET;

    }
    return FLAG_RESET;

}

/**
 * @brief Manages acknowledgment control in I2C communication.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral.
 * @param EnorDi I2C_ACK_ENABLE to enable acknowledgment, I2C_ACK_DISABLE to disable acknowledgment.
 */
void I2C_ManageAcking( I2C_Handle_t *pI2CHandle, uint8_t EnorDi ) {

    if (EnorDi == I2C_ACK_ENABLE ) {

        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 |= (1 << I2C_CR1_ACK ) ;

    } else {

        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR1 &= ~(unsigned int)(1 << I2C_CR1_ACK ) ;
    }
}

/**
 * @brief Enables or disables callback events for the I2C slave.
 *
 * @param I2Cx_RegisterControl[pI2CHandle->I2cIndex] Pointer to the I2C peripheral.
 * @param EnorDi ENABLE to enable callback events, DISABLE to disable.
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_Handle_t *pI2CHandle, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        // Enable the necessary callback events
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= (1 << I2C_CR2_ITBUFEN);
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= (1 << I2C_CR2_ITERREN);
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 |= (1 << I2C_CR2_ITEVTEN);
    } else {
        // Disable the callback events
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 &= ~(unsigned int)(1 << I2C_CR2_ITBUFEN);
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 &= ~(unsigned int)(1 << I2C_CR2_ITERREN);
        I2Cx_RegisterControl[pI2CHandle->I2cIndex]->CR2 &= ~(unsigned int)(1 << I2C_CR2_ITEVTEN);
    }
}






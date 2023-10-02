/**
 * @file 010i2c_master_tx_testing.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<stdio.h>
#include<string.h>
#include "../drivers/Inc/stm32f407xx.h"

#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68

void I2C1_GPIOInits(void);
void I2C1_Inits (void);
void GPIO_ButtonInit(void);

static void delay (void)
{
    // this will introduce ~200ms  delay when SYSCLK is 16MHZ (RC internal for example)
    for( uint32_t i = 0 ; i < 500000/2 ; i++);

}

static I2C_Handle_t I2C1Handle; // global variable

// some data
static uint8_t some_Data[] = " we are testing I2C master T\n" ; /* arduino sketch written by the wire library
		                                                               which has limitaion transfered or received in
		                                                               single transaction and the limit is 32bytes */

/*
 * PB6 -> SCL
 * PB9 -> SDA
 */
void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins ;
    I2CPins.pGPIOx = GPIOB ;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN ;
    I2CPins.GPIO_PinConfig.GPIO_PinPinOPType = GPIO_OP_TYPE_OD ;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU ;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST ;

    //SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6 ;
    GPIO_Init(&I2CPins);

    //SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7 ; /* don't use GPIO_PIN_NO_9 cuz it gives some glitches after every byte will 
                                                              be transmit caused by the swim(single wired interface module)onboard circuitry */
    GPIO_Init(&I2CPins);


}

void I2C1_Inits (void)
{

    I2C1Handle.pI2Cx = I2C1 ;
    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE ;
    I2C1Handle.I2C_Config.I2C_DeviceAdress = MY_ADDR; 	    // in UM10204 file page 17 there is some reserved addresses
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2 ; // standard mode of serial clock speed up to 100KHZ
    I2C1Handle.I2C_Config.I2C_SCLspeed = I2C_SC_SPEED_SM ;  //  Fm mode tlow/thigh = 2

    I2C_Init(&I2C1Handle) ;

}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t Gpioled,GPIOBtn;

//this is Button gpio configuration

    GPIOBtn.pGPIOx=GPIOA; //GPIO base address

    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;       //an external Button
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //we can use PU if we want , or put a external pull up 22k

    GPIO_Init(&GPIOBtn);



    //this is led gpio configuration

    Gpioled.pGPIOx=GPIOD; //GPIO base address

    Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12; // wa can't use the debugging pins (p269 in RF_M) as GPIO
    Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    Gpioled.GPIO_PinConfig.GPIO_PinPinOPType = GPIO_OP_TYPE_PP;
    Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeripheralClockControl(GPIOD,ENABLE);
    GPIO_Init(&Gpioled);

}
int main(void) {


    GPIO_ButtonInit();

    //i2c pin inits
    I2C1_GPIOInits();

    // i2c peripheral configuration
    I2C1_Inits ();

    // enable the i2c peripheral
    I2C_PeripheralControl ( I2C1, ENABLE ) ;
    while(1) {

        // wait till button is pressed
        while (! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

        // to avoid button de-bouncing related issues 200ms of delay
        delay();

        //send some data to the slave
        I2C_MasterSendData (&I2C1Handle, some_Data, /*(unsigned char )*/ strlen( (char*)some_Data), SLAVE_ADDR,I2C_DISABLE_Sr) ; // uploading the sketch in arduino and in the serial monitor it shows the address of the board
    }

    return 0;
}

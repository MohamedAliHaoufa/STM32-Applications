/**
 * @file main.c
 * @author Mohamed Ali Haoufa
 * @brief 
 * @version 0.1
 * @date 2023-09-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

void delay(void){
	int i;
	for (i=0; i<300000;i++);
}

uint32_t LED_Blinky(int PinNum, char name[10]);

#define RCC_BASE_ADDR			0x40023800UL

#define RCC_CR_REG_ADDR 		RCC_BASE_ADDR

#define RCC_AHB1ENR_REG_OFFSET	0x30UL
#define RCC_AHB1ENR_REG_ADDR 	(RCC_BASE_ADDR + RCC_AHB1ENR_REG_OFFSET)

#define GPIOD_BASE_ADDR 	 	0x40020C00UL
#define GPIOD_MODER_REG_ADDR 	GPIOD_BASE_ADDR
#define GPIOD_ODR_REG_OFFSET 	0x14UL
#define GPIOD_ODR_REG_ADDR	 	(GPIOD_BASE_ADDR + GPIOD_ODR_REG_OFFSET)



int main(void)
{

	uint32_t *pRccAhb1enrReg = (uint32_t*) RCC_AHB1ENR_REG_ADDR;
	uint32_t *GpiodModerReg = (uint32_t*) GPIOD_MODER_REG_ADDR;
	uint32_t *GpiodOdrReg = (uint32_t*) GPIOD_ODR_REG_ADDR;
	// Enable Clocks to GPIOD
	*pRccAhb1enrReg |= (uint32_t)(1<<3);


	// set the mode for GPIOD
	// GPIOD pin 12 13 14 15 General purpose output mode :
	*GpiodModerReg &= ~ (0x3<<24); // clear it first
	*GpiodModerReg |= (0x01<<24);

	*GpiodModerReg &= ~ (0x3<<26); // clear it first
	*GpiodModerReg |= (0x01<<26);

	*GpiodModerReg &= ~ (0x3<<28); // clear it first
	*GpiodModerReg |= (0x01<<28);

	*GpiodModerReg &= ~ (0x3<<30); // clear it first
	*GpiodModerReg |= (0x01<<30);


	*GpiodOdrReg &= ~ (1<<12); // clear it first
	*GpiodOdrReg &= ~ (1<<13); // clear it first
	*GpiodOdrReg &= ~ (1<<14); // clear it first
	*GpiodOdrReg &= ~ (1<<15); // clear it first

    /* Loop forever */
	for(;;){
		int i;
		for(i=0;i<2;i++){
		delay();
		*GpiodOdrReg ^= (1<<12); //toggle led in pin 12 GPIOD *
		delay();
		*GpiodOdrReg ^= (1<<13); //toggle led in pin 13 GPIOD *
		delay();
		*GpiodOdrReg ^= (1<<14); //toggle led in pin 14 GPIOD *
		delay();
		*GpiodOdrReg ^= (1<<15); //toggle led in pin 15 GPIOD *
		}
		for(i=0;i<2;i++){
	    delay();
		*GpiodOdrReg &= ~ (1<<12); //toggle led in pin 12 GPIOD *
		*GpiodOdrReg &= ~ (1<<13); //toggle led in pin 13 GPIOD *
		*GpiodOdrReg &= ~ (1<<14); //toggle led in pin 14 GPIOD *
		*GpiodOdrReg &= ~ (1<<15); //toggle led in pin 15 GPIOD *
		delay();
		*GpiodOdrReg |= (1<<12); //toggle led in pin 12 GPIOD *
		*GpiodOdrReg |= (1<<13); //toggle led in pin 13 GPIOD *
		*GpiodOdrReg |= (1<<14); //toggle led in pin 14 GPIOD *
		*GpiodOdrReg |= (1<<15); //toggle led in pin 15 GPIOD *
		}
	}

}



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

#define RCC_BASE_ADDR			0x40023800UL

#define RCC_CR_REG_ADDR 		RCC_BASE_ADDR

#define RCC_CFGR_REG_OFFSET 	0x08UL
#define RCC_CFGR_REG_ADDR 		(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)

#define RCC_AHB1ENR_REG_OFFSET	0x30UL
#define RCC_AHB1ENR_REG_ADDR 	(RCC_BASE_ADDR + RCC_AHB1ENR_REG_OFFSET)

#define GPIOA_BASE_ADDR 		0x40020000UL
#define GPIOA_MODER_REG_ADDR 	GPIOA_BASE_ADDR
#define GPIOA_ODR_REG_OFFSET 	0x14UL
#define GPIOA_ODR_REG_ADDR	 	(GPIOA_BASE_ADDR+GPIOA_ODR_REG_OFFSET)
#define GPIOA_AFRH_REG_OFFSET 	0x24UL
#define GPIOA_AFRH_REG_ADDR	 	(GPIOA_BASE_ADDR+GPIOA_AFRH_REG_OFFSET)

int main(void)
{

	uint32_t *pRccCfgrReg = (uint32_t*) RCC_CFGR_REG_ADDR;
	uint32_t *pRccAhb1enrReg = (uint32_t*) RCC_AHB1ENR_REG_ADDR;

	uint32_t* GpioaModerReg = (uint32_t*) GPIOA_MODER_REG_ADDR;
	uint32_t* GpioaAfrhReg = (uint32_t*) GPIOA_AFRH_REG_ADDR;

	//1. Configure the RCC_CFGR MCO1 bit fields to select HSI as a clock source
	*pRccCfgrReg &= ~ (0x03<<21); // clear 21 and 22 bit

	// choose the value of the MCO1 prescaler divide 16MHZ of HSI by/4 = expect 4MHZ
	*pRccCfgrReg &= ~ (1<<24);
	*pRccCfgrReg |= (1<<25);
	*pRccCfgrReg |= (1<<26);

	// Enable Clocks to GPIOA
	*pRccAhb1enrReg |= (1<<0);

	//set the mode for GPIOA
	*GpioaModerReg &= ~ (0x3<<16); // clear it first
	*GpioaModerReg |= (0x02<<16);

	// choose mode AF0 of PIN8 GPIOA (MCO1 pin) to output the HSI clock after the MCO1 prescaler
    *GpioaAfrhReg &= ~ (0x0F<<0);

    /* Loop forever */
	for(;;);
}


void CAN1_TX_IRQHandler(void){
	printf("Hello World");
}

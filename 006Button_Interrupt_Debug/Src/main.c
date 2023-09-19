/**
 * @file 001led_Toggle.c
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

#define RCC_APB2ENR_REG_OFFSET	0x44UL
#define RCC_APB2ENR_REG_ADDR 	(RCC_BASE_ADDR + RCC_APB2ENR_REG_OFFSET)

#define EXTI_BASE_ADDR			0x40013C00UL

#define EXTI_IMR_REG_OFFSET 	0x00UL
#define EXTI_IMR_REG_ADDR 		(EXTI_BASE_ADDR + EXTI_IMR_REG_OFFSET)

#define EXTI_RTSR_REG_OFFSET 	0x08UL
#define EXTI_RTSR_REG_ADDR 		(EXTI_BASE_ADDR + EXTI_RTSR_REG_OFFSET)

#define EXTI_FTSR_REG_OFFSET 	0x0CUL
#define EXTI_FTSR_REG_ADDR 		(EXTI_BASE_ADDR + EXTI_FTSR_REG_OFFSET)

#define EXTI_PR_REG_OFFSET 		0x14UL
#define EXTI_PR_REG_ADDR 		(EXTI_BASE_ADDR + EXTI_PR_REG_OFFSET)

#define NVIC_IRQ_ENABLE_BASE_ADDR 0xE000E100UL

#define GPIOA_BASE_ADDR 		0x40020000UL

#define GPIOA_MODER_REG_ADDR 	GPIOA_BASE_ADDR

#define GPIOA_ODR_REG_OFFSET 	0x14UL
#define GPIOA_ODR_REG_ADDR	 	(GPIOA_BASE_ADDR+GPIOA_ODR_REG_OFFSET)

#define GPIOA_AFRH_REG_OFFSET 	0x24UL
#define GPIOA_AFRH_REG_ADDR	 	(GPIOA_BASE_ADDR+GPIOA_AFRH_REG_OFFSET)

// global shared variable between code and ISR
uint8_t volatile g_button_pressed = 0;
uint32_t g_button_pressed_count = 0;

void button_init(void);

	uint32_t volatile *pRccCrReg = (uint32_t*) RCC_CR_REG_ADDR;
	uint32_t volatile *pRccCfgrReg = (uint32_t*) RCC_CFGR_REG_ADDR;
	uint32_t volatile *pRccAhb1enrReg = (uint32_t*) RCC_AHB1ENR_REG_ADDR;
	uint32_t volatile *pRccApb2enrReg = (uint32_t*) RCC_APB2ENR_REG_ADDR;

	uint32_t volatile *GpioaModerReg = (uint32_t*) GPIOA_MODER_REG_ADDR;

	uint32_t volatile *pEXTIMaskReg = (uint32_t*) EXTI_IMR_REG_ADDR;
	uint32_t volatile *pEXTIRisingEdgeCtrlReg = (uint32_t*) EXTI_RTSR_REG_ADDR;
	uint32_t volatile *pEXTIPendingReg = (uint32_t*) EXTI_PR_REG_ADDR;

	uint32_t volatile *pNVICIRQEnReg = (uint32_t*) NVIC_IRQ_ENABLE_BASE_ADDR;

int main(void)
{
	button_init();

	while(1){
		// Disable Interrupt
		*pEXTIMaskReg &= ~(1<<0);
		if(g_button_pressed){
			// some delay until button debouncing gets over
			for (uint32_t i; i<500000/2; i++);
			g_button_pressed_count++;
			printf("Button is pressed : %lu\n",g_button_pressed_count);
			g_button_pressed = 0;
		}
		// Enable Interrupt
		*pEXTIMaskReg |= (1<<0);

	}
}

void button_init(void){
	// GPIOA clock enable
	*pRccAhb1enrReg |= (1<<0);

	// syscfg clock enable
	*pRccApb2enrReg |= (1<<14);

	// Edge detection configuration
	*pEXTIRisingEdgeCtrlReg |= (1<<0);

	// EXTI interrupt enable
	*pEXTIMaskReg |= (1<<0);

	// NVIC IRQ enable
	*pNVICIRQEnReg |= (1<<6);
}

// This the Button Interrupt Handler
void EXTI0_IRQHandler(void){
	static int counter = 0;
	counter++;
	printf("The EXTI0 Interrupt has been executed : %d !!\n", counter);

	// Make this flag SET . if button pressed
	g_button_pressed = 1;

	// clearing of EXTI Interrupt pending bit
	*pEXTIPendingReg |= (1<<0);

}

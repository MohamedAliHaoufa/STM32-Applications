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

#define SRAM_ADDR	0x20000004UL

int main(void)
{
	uint32_t value = 0;
	uint32_t volatile *p = (uint32_t*) SRAM_ADDR;

	while (1){
		value = *p;
		if(value) break;
	}
	while(1);
}

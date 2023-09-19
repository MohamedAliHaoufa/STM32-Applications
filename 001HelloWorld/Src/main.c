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

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include<stdio.h>

int main(void)
{
	// start debug session , go for window -> show view -> SWV -> SWV ITM Data Console
	// configure trace (left of red button) -> Check "ITM Stimulus Port 0" -> Red button to start trace
	// Resume (F8 or the (YellowGreen->)arrow) and you'll get the print-f message.
	printf("Hello World\n");

	  /* this will result to an "Instruction fault exception or/ invalid op-code exception" :
	 void (* jump_addr) (void); // function pointer
	 jump_addr= (void *) 0x20000009; // pointer point to RAM address
	 jump_addr(); // jump to execute in RAM address
     */

    /* Loop forever */
	for(;;);
}

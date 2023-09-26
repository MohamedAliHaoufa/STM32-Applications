################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/UART/hal_uart.c 

OBJS += \
./MCAL_Layer/UART/hal_uart.o 

C_DEPS += \
./MCAL_Layer/UART/hal_uart.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/UART/%.o MCAL_Layer/UART/%.su MCAL_Layer/UART/%.cyclo: ../MCAL_Layer/UART/%.c MCAL_Layer/UART/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-UART

clean-MCAL_Layer-2f-UART:
	-$(RM) ./MCAL_Layer/UART/hal_uart.cyclo ./MCAL_Layer/UART/hal_uart.d ./MCAL_Layer/UART/hal_uart.o ./MCAL_Layer/UART/hal_uart.su

.PHONY: clean-MCAL_Layer-2f-UART


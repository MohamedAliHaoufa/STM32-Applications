################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/RCC/hal_rcc.c 

OBJS += \
./MCAL_Layer/RCC/hal_rcc.o 

C_DEPS += \
./MCAL_Layer/RCC/hal_rcc.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/RCC/%.o MCAL_Layer/RCC/%.su MCAL_Layer/RCC/%.cyclo: ../MCAL_Layer/RCC/%.c MCAL_Layer/RCC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-RCC

clean-MCAL_Layer-2f-RCC:
	-$(RM) ./MCAL_Layer/RCC/hal_rcc.cyclo ./MCAL_Layer/RCC/hal_rcc.d ./MCAL_Layer/RCC/hal_rcc.o ./MCAL_Layer/RCC/hal_rcc.su

.PHONY: clean-MCAL_Layer-2f-RCC


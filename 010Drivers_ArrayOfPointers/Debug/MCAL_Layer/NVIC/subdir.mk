################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/NVIC/mcal_nvic.c 

OBJS += \
./MCAL_Layer/NVIC/mcal_nvic.o 

C_DEPS += \
./MCAL_Layer/NVIC/mcal_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/NVIC/%.o MCAL_Layer/NVIC/%.su MCAL_Layer/NVIC/%.cyclo: ../MCAL_Layer/NVIC/%.c MCAL_Layer/NVIC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-NVIC

clean-MCAL_Layer-2f-NVIC:
	-$(RM) ./MCAL_Layer/NVIC/mcal_nvic.cyclo ./MCAL_Layer/NVIC/mcal_nvic.d ./MCAL_Layer/NVIC/mcal_nvic.o ./MCAL_Layer/NVIC/mcal_nvic.su

.PHONY: clean-MCAL_Layer-2f-NVIC


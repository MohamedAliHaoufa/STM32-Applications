################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/GPIO/hal_gpio.c 

OBJS += \
./MCAL_Layer/GPIO/hal_gpio.o 

C_DEPS += \
./MCAL_Layer/GPIO/hal_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/GPIO/%.o MCAL_Layer/GPIO/%.su MCAL_Layer/GPIO/%.cyclo: ../MCAL_Layer/GPIO/%.c MCAL_Layer/GPIO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-GPIO

clean-MCAL_Layer-2f-GPIO:
	-$(RM) ./MCAL_Layer/GPIO/hal_gpio.cyclo ./MCAL_Layer/GPIO/hal_gpio.d ./MCAL_Layer/GPIO/hal_gpio.o ./MCAL_Layer/GPIO/hal_gpio.su

.PHONY: clean-MCAL_Layer-2f-GPIO


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP_Layer/Toggle_LEDs/application.c 

OBJS += \
./APP_Layer/Toggle_LEDs/application.o 

C_DEPS += \
./APP_Layer/Toggle_LEDs/application.d 


# Each subdirectory must supply rules for building sources it contributes
APP_Layer/Toggle_LEDs/%.o APP_Layer/Toggle_LEDs/%.su APP_Layer/Toggle_LEDs/%.cyclo: ../APP_Layer/Toggle_LEDs/%.c APP_Layer/Toggle_LEDs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-APP_Layer-2f-Toggle_LEDs

clean-APP_Layer-2f-Toggle_LEDs:
	-$(RM) ./APP_Layer/Toggle_LEDs/application.cyclo ./APP_Layer/Toggle_LEDs/application.d ./APP_Layer/Toggle_LEDs/application.o ./APP_Layer/Toggle_LEDs/application.su

.PHONY: clean-APP_Layer-2f-Toggle_LEDs


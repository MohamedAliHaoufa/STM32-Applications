################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP_Layer/Interrupt_Button_Toggle_LEDs/application2.c 

OBJS += \
./APP_Layer/Interrupt_Button_Toggle_LEDs/application2.o 

C_DEPS += \
./APP_Layer/Interrupt_Button_Toggle_LEDs/application2.d 


# Each subdirectory must supply rules for building sources it contributes
APP_Layer/Interrupt_Button_Toggle_LEDs/%.o APP_Layer/Interrupt_Button_Toggle_LEDs/%.su APP_Layer/Interrupt_Button_Toggle_LEDs/%.cyclo: ../APP_Layer/Interrupt_Button_Toggle_LEDs/%.c APP_Layer/Interrupt_Button_Toggle_LEDs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-APP_Layer-2f-Interrupt_Button_Toggle_LEDs

clean-APP_Layer-2f-Interrupt_Button_Toggle_LEDs:
	-$(RM) ./APP_Layer/Interrupt_Button_Toggle_LEDs/application2.cyclo ./APP_Layer/Interrupt_Button_Toggle_LEDs/application2.d ./APP_Layer/Interrupt_Button_Toggle_LEDs/application2.o ./APP_Layer/Interrupt_Button_Toggle_LEDs/application2.su

.PHONY: clean-APP_Layer-2f-Interrupt_Button_Toggle_LEDs


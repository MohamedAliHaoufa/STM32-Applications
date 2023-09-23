################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECU_Layer/Keypad/ecu_keypad.c 

OBJS += \
./ECU_Layer/Keypad/ecu_keypad.o 

C_DEPS += \
./ECU_Layer/Keypad/ecu_keypad.d 


# Each subdirectory must supply rules for building sources it contributes
ECU_Layer/Keypad/%.o ECU_Layer/Keypad/%.su ECU_Layer/Keypad/%.cyclo: ../ECU_Layer/Keypad/%.c ECU_Layer/Keypad/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ECU_Layer-2f-Keypad

clean-ECU_Layer-2f-Keypad:
	-$(RM) ./ECU_Layer/Keypad/ecu_keypad.cyclo ./ECU_Layer/Keypad/ecu_keypad.d ./ECU_Layer/Keypad/ecu_keypad.o ./ECU_Layer/Keypad/ecu_keypad.su

.PHONY: clean-ECU_Layer-2f-Keypad


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECU_Layer/LCD/ecu_lcd.c 

OBJS += \
./ECU_Layer/LCD/ecu_lcd.o 

C_DEPS += \
./ECU_Layer/LCD/ecu_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
ECU_Layer/LCD/%.o ECU_Layer/LCD/%.su ECU_Layer/LCD/%.cyclo: ../ECU_Layer/LCD/%.c ECU_Layer/LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ECU_Layer-2f-LCD

clean-ECU_Layer-2f-LCD:
	-$(RM) ./ECU_Layer/LCD/ecu_lcd.cyclo ./ECU_Layer/LCD/ecu_lcd.d ./ECU_Layer/LCD/ecu_lcd.o ./ECU_Layer/LCD/ecu_lcd.su

.PHONY: clean-ECU_Layer-2f-LCD


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECU_Layer/ecu_layer_init.c 

OBJS += \
./ECU_Layer/ecu_layer_init.o 

C_DEPS += \
./ECU_Layer/ecu_layer_init.d 


# Each subdirectory must supply rules for building sources it contributes
ECU_Layer/%.o ECU_Layer/%.su ECU_Layer/%.cyclo: ../ECU_Layer/%.c ECU_Layer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ECU_Layer

clean-ECU_Layer:
	-$(RM) ./ECU_Layer/ecu_layer_init.cyclo ./ECU_Layer/ecu_layer_init.d ./ECU_Layer/ecu_layer_init.o ./ECU_Layer/ecu_layer_init.su

.PHONY: clean-ECU_Layer


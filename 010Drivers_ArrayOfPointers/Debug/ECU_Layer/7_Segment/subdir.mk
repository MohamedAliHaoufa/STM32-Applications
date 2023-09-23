################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECU_Layer/7_Segment/ecu_7_segment.c 

OBJS += \
./ECU_Layer/7_Segment/ecu_7_segment.o 

C_DEPS += \
./ECU_Layer/7_Segment/ecu_7_segment.d 


# Each subdirectory must supply rules for building sources it contributes
ECU_Layer/7_Segment/%.o ECU_Layer/7_Segment/%.su ECU_Layer/7_Segment/%.cyclo: ../ECU_Layer/7_Segment/%.c ECU_Layer/7_Segment/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ECU_Layer-2f-7_Segment

clean-ECU_Layer-2f-7_Segment:
	-$(RM) ./ECU_Layer/7_Segment/ecu_7_segment.cyclo ./ECU_Layer/7_Segment/ecu_7_segment.d ./ECU_Layer/7_Segment/ecu_7_segment.o ./ECU_Layer/7_Segment/ecu_7_segment.su

.PHONY: clean-ECU_Layer-2f-7_Segment


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/S7_Segment/S7_Segment.c 

OBJS += \
./HAL/S7_Segment/S7_Segment.o 

C_DEPS += \
./HAL/S7_Segment/S7_Segment.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/S7_Segment/%.o HAL/S7_Segment/%.su HAL/S7_Segment/%.cyclo: ../HAL/S7_Segment/%.c HAL/S7_Segment/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-HAL-2f-S7_Segment

clean-HAL-2f-S7_Segment:
	-$(RM) ./HAL/S7_Segment/S7_Segment.cyclo ./HAL/S7_Segment/S7_Segment.d ./HAL/S7_Segment/S7_Segment.o ./HAL/S7_Segment/S7_Segment.su

.PHONY: clean-HAL-2f-S7_Segment


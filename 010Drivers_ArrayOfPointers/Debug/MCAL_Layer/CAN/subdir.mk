################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/CAN/hal_can.c 

OBJS += \
./MCAL_Layer/CAN/hal_can.o 

C_DEPS += \
./MCAL_Layer/CAN/hal_can.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/CAN/%.o MCAL_Layer/CAN/%.su MCAL_Layer/CAN/%.cyclo: ../MCAL_Layer/CAN/%.c MCAL_Layer/CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-CAN

clean-MCAL_Layer-2f-CAN:
	-$(RM) ./MCAL_Layer/CAN/hal_can.cyclo ./MCAL_Layer/CAN/hal_can.d ./MCAL_Layer/CAN/hal_can.o ./MCAL_Layer/CAN/hal_can.su

.PHONY: clean-MCAL_Layer-2f-CAN


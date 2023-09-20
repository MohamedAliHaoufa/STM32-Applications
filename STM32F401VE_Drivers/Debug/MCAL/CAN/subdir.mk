################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/CAN/CAN.c 

OBJS += \
./MCAL/CAN/CAN.o 

C_DEPS += \
./MCAL/CAN/CAN.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/CAN/%.o MCAL/CAN/%.su MCAL/CAN/%.cyclo: ../MCAL/CAN/%.c MCAL/CAN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL-2f-CAN

clean-MCAL-2f-CAN:
	-$(RM) ./MCAL/CAN/CAN.cyclo ./MCAL/CAN/CAN.d ./MCAL/CAN/CAN.o ./MCAL/CAN/CAN.su

.PHONY: clean-MCAL-2f-CAN


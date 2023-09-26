################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/NOT\ YET/DAC/DAC.c 

OBJS += \
./MCAL/NOT\ YET/DAC/DAC.o 

C_DEPS += \
./MCAL/NOT\ YET/DAC/DAC.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/NOT\ YET/DAC/DAC.o: ../MCAL/NOT\ YET/DAC/DAC.c MCAL/NOT\ YET/DAC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"MCAL/NOT YET/DAC/DAC.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL-2f-NOT-20-YET-2f-DAC

clean-MCAL-2f-NOT-20-YET-2f-DAC:
	-$(RM) ./MCAL/NOT\ YET/DAC/DAC.cyclo ./MCAL/NOT\ YET/DAC/DAC.d ./MCAL/NOT\ YET/DAC/DAC.o ./MCAL/NOT\ YET/DAC/DAC.su

.PHONY: clean-MCAL-2f-NOT-20-YET-2f-DAC


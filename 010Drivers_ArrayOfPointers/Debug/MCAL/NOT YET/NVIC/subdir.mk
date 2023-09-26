################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/NOT\ YET/NVIC/NVIC.c 

OBJS += \
./MCAL/NOT\ YET/NVIC/NVIC.o 

C_DEPS += \
./MCAL/NOT\ YET/NVIC/NVIC.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/NOT\ YET/NVIC/NVIC.o: ../MCAL/NOT\ YET/NVIC/NVIC.c MCAL/NOT\ YET/NVIC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"MCAL/NOT YET/NVIC/NVIC.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL-2f-NOT-20-YET-2f-NVIC

clean-MCAL-2f-NOT-20-YET-2f-NVIC:
	-$(RM) ./MCAL/NOT\ YET/NVIC/NVIC.cyclo ./MCAL/NOT\ YET/NVIC/NVIC.d ./MCAL/NOT\ YET/NVIC/NVIC.o ./MCAL/NOT\ YET/NVIC/NVIC.su

.PHONY: clean-MCAL-2f-NOT-20-YET-2f-NVIC


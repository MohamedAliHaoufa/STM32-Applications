################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/NOT\ YET/SPI/SPI.c 

OBJS += \
./MCAL_Layer/NOT\ YET/SPI/SPI.o 

C_DEPS += \
./MCAL_Layer/NOT\ YET/SPI/SPI.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/NOT\ YET/SPI/SPI.o: ../MCAL_Layer/NOT\ YET/SPI/SPI.c MCAL_Layer/NOT\ YET/SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"MCAL_Layer/NOT YET/SPI/SPI.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-NOT-20-YET-2f-SPI

clean-MCAL_Layer-2f-NOT-20-YET-2f-SPI:
	-$(RM) ./MCAL_Layer/NOT\ YET/SPI/SPI.cyclo ./MCAL_Layer/NOT\ YET/SPI/SPI.d ./MCAL_Layer/NOT\ YET/SPI/SPI.o ./MCAL_Layer/NOT\ YET/SPI/SPI.su

.PHONY: clean-MCAL_Layer-2f-NOT-20-YET-2f-SPI


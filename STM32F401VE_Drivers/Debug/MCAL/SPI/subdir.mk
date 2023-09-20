################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/SPI/SPI.c 

OBJS += \
./MCAL/SPI/SPI.o 

C_DEPS += \
./MCAL/SPI/SPI.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/SPI/%.o MCAL/SPI/%.su MCAL/SPI/%.cyclo: ../MCAL/SPI/%.c MCAL/SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL-2f-SPI

clean-MCAL-2f-SPI:
	-$(RM) ./MCAL/SPI/SPI.cyclo ./MCAL/SPI/SPI.d ./MCAL/SPI/SPI.o ./MCAL/SPI/SPI.su

.PHONY: clean-MCAL-2f-SPI


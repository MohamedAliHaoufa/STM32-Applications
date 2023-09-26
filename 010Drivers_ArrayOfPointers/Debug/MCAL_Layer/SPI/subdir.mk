################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/SPI/hal_spi.c 

OBJS += \
./MCAL_Layer/SPI/hal_spi.o 

C_DEPS += \
./MCAL_Layer/SPI/hal_spi.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/SPI/%.o MCAL_Layer/SPI/%.su MCAL_Layer/SPI/%.cyclo: ../MCAL_Layer/SPI/%.c MCAL_Layer/SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-SPI

clean-MCAL_Layer-2f-SPI:
	-$(RM) ./MCAL_Layer/SPI/hal_spi.cyclo ./MCAL_Layer/SPI/hal_spi.d ./MCAL_Layer/SPI/hal_spi.o ./MCAL_Layer/SPI/hal_spi.su

.PHONY: clean-MCAL_Layer-2f-SPI


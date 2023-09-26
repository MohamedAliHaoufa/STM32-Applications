################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/I2C/hal_i2c.c 

OBJS += \
./MCAL_Layer/I2C/hal_i2c.o 

C_DEPS += \
./MCAL_Layer/I2C/hal_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/I2C/%.o MCAL_Layer/I2C/%.su MCAL_Layer/I2C/%.cyclo: ../MCAL_Layer/I2C/%.c MCAL_Layer/I2C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-I2C

clean-MCAL_Layer-2f-I2C:
	-$(RM) ./MCAL_Layer/I2C/hal_i2c.cyclo ./MCAL_Layer/I2C/hal_i2c.d ./MCAL_Layer/I2C/hal_i2c.o ./MCAL_Layer/I2C/hal_i2c.su

.PHONY: clean-MCAL_Layer-2f-I2C


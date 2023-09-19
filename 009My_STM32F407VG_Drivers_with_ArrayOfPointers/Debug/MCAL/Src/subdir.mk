################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL/Src/GPIO.c 

OBJS += \
./MCAL/Src/GPIO.o 

C_DEPS += \
./MCAL/Src/GPIO.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL/Src/%.o MCAL/Src/%.su MCAL/Src/%.cyclo: ../MCAL/Src/%.c MCAL/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL-2f-Src

clean-MCAL-2f-Src:
	-$(RM) ./MCAL/Src/GPIO.cyclo ./MCAL/Src/GPIO.d ./MCAL/Src/GPIO.o ./MCAL/Src/GPIO.su

.PHONY: clean-MCAL-2f-Src


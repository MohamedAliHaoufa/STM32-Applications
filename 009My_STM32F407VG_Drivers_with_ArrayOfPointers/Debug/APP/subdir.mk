################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/main.c \
../APP/syscalls.c \
../APP/sysmem.c 

OBJS += \
./APP/main.o \
./APP/syscalls.o \
./APP/sysmem.o 

C_DEPS += \
./APP/main.d \
./APP/syscalls.d \
./APP/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
APP/%.o APP/%.su APP/%.cyclo: ../APP/%.c APP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-APP

clean-APP:
	-$(RM) ./APP/main.cyclo ./APP/main.d ./APP/main.o ./APP/main.su ./APP/syscalls.cyclo ./APP/syscalls.d ./APP/syscalls.o ./APP/syscalls.su ./APP/sysmem.cyclo ./APP/sysmem.d ./APP/sysmem.o ./APP/sysmem.su

.PHONY: clean-APP


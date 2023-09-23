################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP_Layer/application.c \
../APP_Layer/syscalls.c \
../APP_Layer/sysmem.c 

OBJS += \
./APP_Layer/application.o \
./APP_Layer/syscalls.o \
./APP_Layer/sysmem.o 

C_DEPS += \
./APP_Layer/application.d \
./APP_Layer/syscalls.d \
./APP_Layer/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
APP_Layer/%.o APP_Layer/%.su APP_Layer/%.cyclo: ../APP_Layer/%.c APP_Layer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-APP_Layer

clean-APP_Layer:
	-$(RM) ./APP_Layer/application.cyclo ./APP_Layer/application.d ./APP_Layer/application.o ./APP_Layer/application.su ./APP_Layer/syscalls.cyclo ./APP_Layer/syscalls.d ./APP_Layer/syscalls.o ./APP_Layer/syscalls.su ./APP_Layer/sysmem.cyclo ./APP_Layer/sysmem.d ./APP_Layer/sysmem.o ./APP_Layer/sysmem.su

.PHONY: clean-APP_Layer


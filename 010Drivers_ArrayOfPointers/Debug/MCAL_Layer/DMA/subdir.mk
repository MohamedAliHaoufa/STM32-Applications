################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/DMA/hal_dma.c 

OBJS += \
./MCAL_Layer/DMA/hal_dma.o 

C_DEPS += \
./MCAL_Layer/DMA/hal_dma.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/DMA/%.o MCAL_Layer/DMA/%.su MCAL_Layer/DMA/%.cyclo: ../MCAL_Layer/DMA/%.c MCAL_Layer/DMA/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-DMA

clean-MCAL_Layer-2f-DMA:
	-$(RM) ./MCAL_Layer/DMA/hal_dma.cyclo ./MCAL_Layer/DMA/hal_dma.d ./MCAL_Layer/DMA/hal_dma.o ./MCAL_Layer/DMA/hal_dma.su

.PHONY: clean-MCAL_Layer-2f-DMA


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCAL_Layer/TIM/mcal_tim.c 

OBJS += \
./MCAL_Layer/TIM/mcal_tim.o 

C_DEPS += \
./MCAL_Layer/TIM/mcal_tim.d 


# Each subdirectory must supply rules for building sources it contributes
MCAL_Layer/TIM/%.o MCAL_Layer/TIM/%.su MCAL_Layer/TIM/%.cyclo: ../MCAL_Layer/TIM/%.c MCAL_Layer/TIM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MCAL_Layer-2f-TIM

clean-MCAL_Layer-2f-TIM:
	-$(RM) ./MCAL_Layer/TIM/mcal_tim.cyclo ./MCAL_Layer/TIM/mcal_tim.d ./MCAL_Layer/TIM/mcal_tim.o ./MCAL_Layer/TIM/mcal_tim.su

.PHONY: clean-MCAL_Layer-2f-TIM


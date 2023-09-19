################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include MCAL/GPIO/subdir.mk
-include HAL/S7_Segment/subdir.mk
-include HAL/LCD/subdir.mk
-include HAL/Keypad/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := 009My_STM32F407VG_Drivers_with_ArrayOfPointers
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
009My_STM32F407VG_Drivers_with_ArrayOfPointers.elf \

MAP_FILES += \
009My_STM32F407VG_Drivers_with_ArrayOfPointers.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
009My_STM32F407VG_Drivers_with_ArrayOfPointers.list \

OBJCOPY_HEX += \
009My_STM32F407VG_Drivers_with_ArrayOfPointers.hex \


# All Target
all: main-build

# Main-build Target
main-build: 009My_STM32F407VG_Drivers_with_ArrayOfPointers.elf secondary-outputs

# Tool invocations
009My_STM32F407VG_Drivers_with_ArrayOfPointers.elf 009My_STM32F407VG_Drivers_with_ArrayOfPointers.map: $(OBJS) $(USER_OBJS) /home/mohamed/Desktop/Embedded\ \ Martin\ Schröder\ Courses/Courses/Kiran\ Nayak_slideshare/MCU1\ course/MCU1/009My_STM32F407VG_Drivers_with_ArrayOfPointers/STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "009My_STM32F407VG_Drivers_with_ArrayOfPointers.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/mohamed/Desktop/Embedded  Martin Schröder Courses/Courses/Kiran Nayak_slideshare/MCU1 course/MCU1/009My_STM32F407VG_Drivers_with_ArrayOfPointers/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="009My_STM32F407VG_Drivers_with_ArrayOfPointers.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

009My_STM32F407VG_Drivers_with_ArrayOfPointers.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "009My_STM32F407VG_Drivers_with_ArrayOfPointers.list"
	@echo 'Finished building: $@'
	@echo ' '

009My_STM32F407VG_Drivers_with_ArrayOfPointers.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "009My_STM32F407VG_Drivers_with_ArrayOfPointers.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) 009My_STM32F407VG_Drivers_with_ArrayOfPointers.elf 009My_STM32F407VG_Drivers_with_ArrayOfPointers.hex 009My_STM32F407VG_Drivers_with_ArrayOfPointers.list 009My_STM32F407VG_Drivers_with_ArrayOfPointers.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets

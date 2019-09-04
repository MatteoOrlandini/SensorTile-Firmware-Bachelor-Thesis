################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/STM32_USBD_Library/Core/subdir.mk
-include Middlewares/STM32_USBD_Library/Class/CDC/subdir.mk
-include Middlewares/STM32_BlueNRG/Utils/subdir.mk
-include Middlewares/STM32_BlueNRG/Interface/subdir.mk
-include Middlewares/STM32_BlueNRG/HCI/Controller/subdir.mk
-include Middlewares/STM32_BlueNRG/HCI/subdir.mk
-include Middlewares/MetaDataManager/subdir.mk
-include Middlewares/FatFs/Options/subdir.mk
-include Middlewares/FatFs/Drivers/subdir.mk
-include Middlewares/FatFs/Core/subdir.mk
-include FS-SNS-ALLMEMS1/User/subdir.mk
-include FS-SNS-ALLMEMS1/SW4STM32/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/SensorTile/subdir.mk
-include Drivers/BSP/Components/subdir.mk
-include Doc/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: ALLMEMS1_ST.elf

# Tool invocations
ALLMEMS1_ST.elf: $(OBJS) $(USER_OBJS) ../LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -L"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionAR_Library/Lib" -L"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionCP_Library/Lib" -L"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionFX_Library/Lib" -L"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionGR_Library/Lib" -L"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueVoiceADPCM_Library/Lib"  -specs=nano.specs -T"../LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "ALLMEMS1_ST.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary:'
	arm-none-eabi-objcopy -O binary "ALLMEMS1_ST.elf" "ALLMEMS1_ST.bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets

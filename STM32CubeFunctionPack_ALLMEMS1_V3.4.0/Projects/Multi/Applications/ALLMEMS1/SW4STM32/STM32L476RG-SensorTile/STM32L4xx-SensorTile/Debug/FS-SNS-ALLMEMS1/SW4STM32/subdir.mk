################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/SW4STM32/STM32L476RG-SensorTile/startup_stm32l476xx.s 

C_SRCS += \
/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/Src/syscalls.c 

OBJS += \
./FS-SNS-ALLMEMS1/SW4STM32/startup_stm32l476xx.o \
./FS-SNS-ALLMEMS1/SW4STM32/syscalls.o 

C_DEPS += \
./FS-SNS-ALLMEMS1/SW4STM32/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
FS-SNS-ALLMEMS1/SW4STM32/startup_stm32l476xx.o: /home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/SW4STM32/STM32L476RG-SensorTile/startup_stm32l476xx.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I../../../../Inc -I../../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../../../../../../../Drivers/BSP/STM32L4xx_Nucleo -I../../../../../../../../Drivers/CMSIS/Include -I../../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../../Drivers/BSP/Components/hts221 -I../../../../../../../../Drivers/BSP/Components/lis3mdl -I../../../../../../../../Drivers/BSP/Components/lps25hb -I../../../../../../../../Drivers/BSP/Components/lsm6ds0 -I../../../../../../../../Drivers/BSP/Components/lsm6ds3 -I../../../../../../../../Middlewares/ST/STM32_MotionFX_Library/Inc -I../../../../../../../../Middlewares/ST/STM32_MotionFX_Library -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

FS-SNS-ALLMEMS1/SW4STM32/syscalls.o: /home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/Src/syscalls.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c99 -DUSE_HAL_DRIVER '-DSTM32_SENSORTILE=1' -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DALLMEMS1 -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/SensorTile" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/CMSIS/Include" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/Common" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/hts221" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/stc3115" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionAR_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionCP_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionFX_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionGR_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MetaDataManager" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src/drivers"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src/option/syscall.c \
/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src/option/unicode.c 

OBJS += \
./Middlewares/FatFs/Options/syscall.o \
./Middlewares/FatFs/Options/unicode.o 

C_DEPS += \
./Middlewares/FatFs/Options/syscall.d \
./Middlewares/FatFs/Options/unicode.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FatFs/Options/syscall.o: /home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src/option/syscall.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c99 -DUSE_HAL_DRIVER '-DSTM32_SENSORTILE=1' -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DALLMEMS1 -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/SensorTile" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/CMSIS/Include" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/Common" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/hts221" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/stc3115" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionAR_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionCP_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionFX_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionGR_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MetaDataManager" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src/drivers"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FatFs/Options/unicode.o: /home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src/option/unicode.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c99 -DUSE_HAL_DRIVER '-DSTM32_SENSORTILE=1' -DSTM32L476xx -DUSE_STM32L4XX_NUCLEO -DALLMEMS1 -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/SensorTile" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/CMSIS/Include" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/Common" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueNRG/Interface" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lsm6dsm" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/hts221" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lps22hb" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/lsm303agr" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Drivers/BSP/Components/stc3115" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionAR_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionCP_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionFX_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MotionGR_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueVoiceADPCM_Library/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_MetaDataManager" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src" -I"/home/matteo/Scrivania/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/Third_Party/FatFs/src/drivers"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



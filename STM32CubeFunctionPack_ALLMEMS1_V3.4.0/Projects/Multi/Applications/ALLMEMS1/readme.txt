/**
  ******************** (C) COPYRIGHT 2018 STMicroelectronics *******************
  * @file    readme.txt
  * @author  Central LAB
  * @version V3.4.0
  * @date    26-Apr-2018
  * @brief   Description of the Application FW.
  ******************************************************************************
  * Attention
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

Application Description 

 This firmware package includes Components Device Drivers, Board Support Package
 and example application for the following STMicroelectronics elements:
 - X-NUCLEO-IDB04A1/X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards
 - X-NUCLEO-IKS01A1 Expansion board for four MEMS sensor devices:
       HTS221, LPS25H, LSM6DS0, LSM6DS3, LIS3MDL
 - X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
       HTS221, LPS22HB, LSM6DSL, LSM303AGR
 - X-NUCLEO-CCAM02M1 Digital MEMS microphones expansion board
 - NUCLEO-F446RE NUCLEO-F401RE NUCLEO-L476RG Nucleo boards
 - STEVAL-STLKT01V1 (SensorTile) evaluation board that contains the following components:
	. MEMS sensor devices: HTS221, LPS22HB, LSM303AGR, LSM6DSM
	. digital microphone: MP34DT04
	. Gas Gauge IC with alarm output: STC3115
 - STEVAL-BCNKT01V1 (BlueCoin) evaluation board that contains the following components:
	. MEMS sensor devices: LPS22HB, LSM303AGR, LSM6DSM
	. digital microphone: 4 x MP34DT04-C1
 - The MotionFX (iNEMOEngine PRO) suite uses advanced algorithms to integrate outputs
   from multiple MEMS sensors in a smartway, independent of environmental conditions,
   to reach optimal performance. Real-time motion-sensor data fusion is set to significantly
   improve the user experience, increasing accuracy, resolution, stability and response time.
 - MotionAR (iNEMOEngine PRO) software provides real-time activity recognition data 
   using MEMS accelerometer sensor
 - MotionCP (iNEMOEngine PRO) software provides carry Position recognition data using MEMS accelerometer sensor
   (feature not available on NUCLEO-F446RE, NUCLEO-F401RE and STEVAL-BCNKT01V1) 
 - MotionGR (iNEMOEngine PRO) software provides carry Gesture recognition data using MEMS accelerometer sensor
   (feature not available on NUCLEO-F446RE, NUCLEO-F401RE and STEVAL-BCNKT01V1)
 - AcousticSL software provides real-time audio source localization using PCM signal audio
   (feature not available on the STEVAL-STLKT01V1)
 - AcousticBF software provides real-time beam forming software, using the audio signals acquired from two digital MEMS microphones,
   it creates a virtual directional microphone pointing to a fixed direction in space
   (feature not available on STEVAL-STLKT01V1 and NUCLEO-L476RG).
 - BlueVoiceADPCM software enables real-time half-duplex voice-over-Bluetooth low energy communication profile.
   It includes one characteristic for audio transmission and one for synchronization and it is responsible for audio encoding and periodical data
   transmission on Server side and for decoding of received voice data on Client side
 - USB device library (for only STEVAL-STLCS01V1) provides support of multi packet transfer to allow
   sending big amount of data without split them into max packet size transfers.
 - FatFs generic FAT file system module (for only STEVAL-STLCS01V1) provides access the storage devices 
   such as memory card and hard disk.

 The Example application initializes all the Components and Library creating 3 Custom Bluetooth services:
 - The first service exposes all the HW and SW characteristics:
   . HW characteristics:
		. related to MEMS sensor devices: Temperature, Humidity, Pressure, Magnetometer, Gyroscope and Accelleromenter
		  and Microphones Signal Noise dB level.
		. battery Gas Gauge (for only SensorTile) 
   . SW characteristics: the quaternions generated by the MotionFX library in short precision, the activity
     recognized using the MotionAR algorithm, the carry position recognized using the MotionCP algorithm (excluding STM32 Nucleo F4xx),
     the Gesture recognized using the MotionGR algorithm(excluding STM32 Nucleo F4xx), the audio source localization using the AcousticSL software (excluding STEVAL-STLKT01V1)
	 that provides real-time audio source localization, AcousticBF software (Excluding STEVAL-STLKT01V1 and STM32 Nucleo L476) provides real-time beam forming
	 using the audio signals acquired from two digital MEMS microphones.
	 It uses:
		. BlueVoiceADPCM software for real-time half-duplex voice-over-Bluetooth low energy communication profile.
		. FatFs generic FAT file system for SD card data logging for environmental, mens and audio data (for only SensorTile).
		  This functionality on SensorTile board is enabled as default through the code in the line 68 
		    #define ALLMEMS1_ENABLE_SD_CARD_LOGGING
		  on file:
		    Projects\Multi\Applications\ALLMEMS1\Inc\ALLMEMS1_config.h
 - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 - The last Service is used for configuration purpose
 
 For the STEVAL-STLKT01V1, when the Android/iOS device is not connected for more than 20 seconds, the board go on shutdown mode.
 The shutdown mode can be enabled or disabled by means of the macro ENABLE_SHUT_DOWN_MODE
 The accelerometer event can be selected and used to wake-up the board to connect it to Android/iOS again and it can be chosen by
 the constant WakeupSource in the file main.c (The Double Tap event is set as default).
 Through the define RANGE_TIME_WITHOUT_CONNECTED in main.h file it is possible modified this time value.
 
 For NUCLEO boards the example application allows the user to control the initialization phase via UART.
 Launch a terminal application and set the UART port to 460800 bps, 8 bit, No Parity, 1 stop bit.
 For having the same UART functionality on SensorTile board, is necessary to recompile the code uncomment the line 103 
	//#define ALLMEMS1_ENABLE_PRINTF
 on file:
	Projects\Multi\Applications\ALLMEMS1\Inc\ALLMEMS1_config.h
 and if not enabled the SD card data logging functionality.
 This enables the UART that starts with a delay of 10 Seconds for allowing the time to open the UART for looking the initialization phase.

 This example must be used with the related BlueMS Android/iOS application available on Play/itune store (Version 3.6.0 or higher),
 in order to read the sent information by Bluetooth Low Energy protocol

                   ---------------------------------------------
                   | Important Hardware Additional Information |
			       ---------------------------------------------
1) Before to connect X-NUCLEO-IKS01A1 with X-NUCLEO-IDB04A1 (or X-NUCLEO-IDB05A1) expansion board through the Arduino UNO R3 extension connector,
   remove the 0-ohm resistors SB25, SB26 and SB27 onto X-NUCLEO-IKS01A1 board.
2) Before to connect X-NUCLEO-IKS01A2 with X-NUCLEO-CCAM02M1 expansion board through the Arduino UNO R3 extension connector,
   on to X-NUCLEO-IKS01A2 board remove these 0-ohm resistor:
   - for F4xx STM32 Nucleo motherboard remove SB25, SB26 and SB27
   - for L4 STM32 Nucleo motherboard remove SB25 if additional microphones are plugged on to X-NUCLEO-CCA02M1 board.
3) For only L4 STM32 Nucleo motherboard, before to connect the board X-NUCLEO-CCA02M1 with the STM32 L4 Nucleo motherboard through the Morpho connector layout, on to X-NUCLEO-CCA02M1 board:
   - close the solder bridges SB12, SB16 and open the solder bridges SB7, SB15 and SB17
   - if additional microphones are plugged, close the solder bridge SB17.
  
                              -----------------------
                              | VERY IMPORTANT (1): |
                              -----------------------
 1) This example support the Firmware-Over-The-Air (FOTA) update (for only X-NUCLEO-IDB05A1 Bluetooth
 Low energy expansion boards) using the BlueMS Android/iOS application (Version 3.0.0 or higher)
 2) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
 is saved at the beginning of the FLASH (address 0x08000000)
 
 For each IDE (IAR/µVision/System Workbench), and for each platform (NUCLEO-F446RE/NUCLEO-F401RE/NUCLEO-L476RG/SensorTile/BlueCoin),
 there are some scripts *.bat/*.sh that makes the following operations:
 - Full Flash Erase
 - Load the BootLoader on the rigth flash region
 - Load the Program (after the compilation) on the rigth flash region (This could be used for a FOTA)
 - Dump back one single binary that contain BootLoader+Program that could be 
   flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
 - Reset the board
 
                               -----------------------
                               | VERY IMPORTANT (2): |
                               -----------------------
 It's necessary to choose the right Target configuration during the compilation.
 If the code is compiled for IKS01A1 could not run if it's attached the X-NUCLEO-IKS01A2 and vice versa.

                                    ----------
                                    | ISSUE: |
                                    ----------
 - FOTA is not available on STEVAL-BCNKT01V1 (BlueCoin), NUCLEO-F401RE and NUCLEO-F446RE using binary generated by System Workbench for STM32
   (due to flash size constraints).
 - Led features is not enabled for SensorTile when SD card logging feature is enabled and during the recording only
  (The led will blink during the data logging only).
 - A compiler warning is generated from AcousticSL and AcousticBF middlewares when using the library in IAR v 8.x.
   It doesn't affect library performances.
 - For L4 STM32 Nucleo motherboard and STEVAL-STLCS01V1 Board, a compiler warning is generated from MotionFX middleware when using the library in MDK-ARM v 5.24.
   It doesn't affect library performances.

                                --------------------
                                | KNOWN LIMITATION |
                                --------------------
 - Even if FP-SNS-ALLMEMS1 send 100 quaternions/second with Bluetooth, the mobile devices could render only 60 frames/second
 - FOTA does not work when using X-NUCLEO-IDB04A1
 - For NUCLEO-F446RE/NUCLEO-F401RE board, there is an hardware conflict between the boards X-NUCLEO-IKS01A2 and the X-NUCLEO-CCA02M1.
   The hardware features of the LSM6DSL are disabled.
 
 
 Inside the Binary Directory there are the following binaries:
Binary/
+-- STM32F446RE-BlueCoin
    +-- ALLMEMS1_BC.bin     (Program without BootLoader. COULD BE USED     for FOTA)
    +-- ALLMEMS1_BC_BL.bin  (Program with BootLoader.    COULD NOT BE USED for FOTA)
+-- STM32F446RE-Nucleo
¦   +-- ALLMEMS1_IKS01A1_NucleoF446.bin    (Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- ALLMEMS1_IKS01A2_NucleoF446.bin    (Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- ALLMEMS1_IKS01A1_NucleoF446_BL.bin (Program with BootLoader.    COULD NOT BE USED for FOTA)
¦   +-- ALLMEMS1_IKS01A2_NucleoF446_BL.bin (Program with BootLoader.    COULD NOT BE USED for FOTA)
+-- STM32F401RE-Nucleo
¦   +-- ALLMEMS1_IKS01A1_NucleoF401.bin    (Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- ALLMEMS1_IKS01A2_NucleoF401.bin    (Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- ALLMEMS1_IKS01A1_NucleoF401_BL.bin (Program with BootLoader.    COULD NOT BE USED for FOTA)
¦   +-- ALLMEMS1_IKS01A2_NucleoF401_BL.bin (Program with BootLoader.    COULD NOT BE USED for FOTA)
+-- STM32L476RG-Nucleo
¦   +-- ALLMEMS1_IKS01A1_NucleoL476.bin    (Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- ALLMEMS1_IKS01A2_NucleoL476.bin    (Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- ALLMEMS1_IKS01A1_NucleoL476_BL.bin (Program with BootLoader.    COULD NOT BE USED for FOTA)
¦   +-- ALLMEMS1_IKS01A2_NucleoL476_BL.bin (Program with BootLoader.    COULD NOT BE USED for FOTA)
+-- STM32L476RG-SensorTile
    +-- ALLMEMS1_ST.bin     (Program without BootLoader. COULD BE USED     for FOTA)
    +-- ALLMEMS1_ST_BL.bin  (Program with BootLoader.    COULD NOT BE USED for FOTA)

 The Magneto Calibration provided by MotionFX (INEMOEngine PRO) could be saved in FLASH.
 In this way it's not necessary to make again the calibration at every STM32 Nucleo board reset.

 For avoiding accidental erasure of the calibration data, for forcing a new calibration it is necessary to press 
 the blue user button on the STM32 Nucleo board 3 times in less than 2 seconds.

@par Hardware and Software environment

  - This example runs on Sensor expansion board attached to STM32F446RE/STM32F401RE/STM32L476RG devices
    can be easily tailored to any other supported device and development board.
    Or it runs on STEVAL-STLKT01V1 (SensorTile) evaluation board
    
  - This example must be used with the related BlueMS Android/iOS application (Version 3.5.0 or higher) available on Play/itune store,
    in order to read the sent information by Bluetooth Low Energy protocol
    
@par STM32Cube packages:
  - STM32F4xx drivers from STM32CubeF4 V1.21.0
  - STM32L4xx drivers from STM32CubeL4 V1.11.1
@par X-CUBE packages:
  - X-CUBE-BLE1 V3.3.0
  - X-CUBE-MEMS1 V4.4.0 Modified
  - X-CUBE-MEMSMIC1 V3.0.0
@par STEVAL-STLCS01V1:
  - STEVAL-STLCS01V1 V1.3.3
@par STEVAL-BCNKT01V1:
  - STEVAL-BCNKT01V1 V1.2.0

@par How to use it ? 

This package contains projects for 3 IDEs viz. IAR, µVision and System Workbench. 
In order to make the  program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V8.20.2).
 - Open the IAR project file EWARM\STM32F446RE-Nucleo\ALLMEMS1_NF446.eww or EWARM\STM32F401RE-Nucleo\ALLMEMS1_NF401.eww or EWARM\STM32F401RE-Nucleo\ALLMEMS1_NL476.eww or EWARM\STM32L476RG-SensorTile\ALLMEMS1_ST.eww or EWARM\STM32F446RE-BlueCoin\ALLMEMS1_BC.eww.
 - Rebuild all files and run these script that you find on the same directory:
		- For Nucleo F446: CleanALLMEMS1_IAR_IKS01A1_F446.bat or CleanALLMEMS1_IAR_IKS01A2_F446.bat
		- For Nucleo F401: CleanALLMEMS1_IAR_IKS01A1_F401.bat or CleanALLMEMS1_IAR_IKS01A2_F401.bat
		- For Nucleo L476: CleanALLMEMS1_IAR_IKS01A1_L476.bat or CleanALLMEMS1_IAR_IKS01A2_L476.bat
		- For STEVAL-BCNKT01V1: CleanALLMEMS1_IAR_BC.bat
		- For STEVAL-STLKT01V1: CleanALLMEMS1_IAR_ST.bat

For µVision:
 - Open µVision toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.24.2)
 - Open the µVision project file MDK-ARM\STM32F446RE-Nucleo\Project.uvprojx or MDK-ARM\STM32F401RE-Nucleo\Project.uvprojx or MDK-ARM\STM32L476RG-Nucleo\Project.uvprojx or MDK-ARM\STM32L476RG-SensorTile\Project.uvprojx or MDK-ARM\STM32F446RE-BlueCoin\Project.uvprojx
 - Rebuild all files and run these script that you find on the same directory.
		- For Nucleo F446: CleanALLMEMS1_MDK_ARM_IKS01A1_F446.bat or CleanALLMEMS1_MDK_ARM_IKS01A2_F446.bat
		- For Nucleo F401: CleanALLMEMS1_MDK_ARM_IKS01A1_F401.bat or CleanALLMEMS1_MDK_ARM_IKS01A2_F401.bat
		- For Nucleo L476: CleanALLMEMS1_MDK_ARM_IKS01A1_L476.bat or CleanALLMEMS1_MDK_ARM_IKS01A2_L476.bat
		- For STEVAL-BCNKT01V1: CleanALLMEMS1_MDK_ARM_BC.bat
		- For STEVAL-STLKT01V1: CleanALLMEMS1_MDK_ARM_ST.bat
		
For System Workbench:
 - Open System Workbench for STM32 (this firmware has been successfully tested with System Workbench for STM32 Version 2.4.0.201801120948)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be SW4STM32\STM32F446RE-Nucleo\ or SW4STM32\STM32F401RE-Nucleo\ or SW4STM32\STM32L476RG-Nucleo\ or SW4STM32\STM32L476RG-SensorTile or SW4STM32\STM32F446RE-BlueCoin). 
 - Rebuild all files and and run these script that you find on the same directory:
   - if you are on windows and you had installed the STM32 ST-Link utility:
		- For Nucleo F446: CleanALLMEMS1_SW4STM32_IKS01A1_F446.bat or CleanALLMEMS1_SW4STM32_IKS01A2_F446.bat
		- For Nucleo F401: CleanALLMEMS1_SW4STM32_IKS01A1_F401.bat or CleanALLMEMS1_SW4STM32_IKS01A2_F401.bat
		- For Nucleo L476: CleanALLMEMS1_SW4STM32_IKS01A1_L476.bat or CleanALLMEMS1_SW4STM32_IKS01A2_L476.bat
		- For STEVAL-BCNKT01V1: CleanALLMEMS1_SW4STM32_BC.bat
		- For STEVAL-STLKT01V1: CleanALLMEMS1_SW4STM32_ST.bat	
   - Otherwise (Linux/iOS or Windows without the STM32 ST-Link Utility):
		- For Nucleo F446: CleanALLMEMS1_SW4STM32_IKS01A1_F446.sh or CleanALLMEMS1_SW4STM32_IKS01A2_F446.sh
		- For Nucleo F401: CleanALLMEMS1_SW4STM32_IKS01A1_F401.sh or CleanALLMEMS1_SW4STM32_IKS01A2_F401.sh
		- For Nucleo L476: CleanALLMEMS1_SW4STM32_IKS01A1_L476.sh or CleanALLMEMS1_SW4STM32_IKS01A2_L476.sh
		- For STEVAL-BCNKT01V1: CleanALLMEMS1_SW4STM32_BC.sh
		- For STEVAL-STLKT01V1: CleanALLMEMS1_SW4STM32_ST.sh
		
 /******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

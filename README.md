# SensorTile-Firmware-Thesis

## Introduction
This project is part of my bachelor thesis and was born with the aim of developing a firmware in C language to acquire data from motion sensors present on the SensorTile STLCS02V1 platform. The MEMS sensors LSM6DSM (accelerometer + gyroscope) and LSM303AGR (accelerometer + magnetic sensor) allow non-invasive use and extremely low energy consumption. The aim of the project is the transmission of Bluetooth packets containing data from inertial sensors. Measurements obtained from the various components were merged to optimize energy consumption and performance. 

To receive the Bluetooth packets, some [Python scripts](https://github.com/MatteoOrlandini/Bluepy-Python-Thesis) have been written using the [bluepy library](https://github.com/IanHarvey/bluepy) that allows the user to save the detected values. In particular, the pitch and roll angles obtained by the gyroscope and by the accelerometer are calculated by doing data processing on the board using a sensor fusion algorithm and a complementary filter.

During the project, the open source firmware [FP-SNS-ALLMEMS1](https://www.st.com/en/embedded-software/fp-sns-allmems1.html) made available by STMicroelectronics and compatible with the SensorTile was studied, modified and expanded.

## Prereqs

* [System Workbench for STM32](https://www.st.com/en/development-tools/sw4stm32.html)
* Install libncurses5 32 bit

  `sudo add-apt-repository universe`
  
  `sudo apt-get install libncurses5 libncurses5:i386`
  
* Install [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)

* Install [OpenOCD](http://openocd.org/getting-openocd/)

  `sudo apt-get install openocd`

## System Workbench for STM32 configuration

* Rename staric library "libbluevoiceADPCM 200_CM4F_GCC_ot.a" in path [STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueVoiceADPCM_Library/Lib](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/tree/master/STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Middlewares/ST/STM32_BlueVoiceADPCM_Library/Lib) to "libBlueVoiceADPCM_200_CM4F_GCC_ot.a"

* Set `STM32 SENSORTILE = 1` in preprocessor settings

## How to build

This package contains projects for 3 IDEs viz. IAR, µVision and System Workbench. 
In order to make the  program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder installation path is not too in-depth since the toolchain may report errors after building.
		
For System Workbench:
 - Open System Workbench for STM32 (this firmware has been successfully tested with System Workbench for STM32 Version 2.4.0.201801120948)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (SW4STM32\STM32L476RG-SensorTile). 
 - Rebuild all files and and run these script that you find on the same directory:
   - if you are on windows and you had installed the STM32 ST-Link utility:
		- For STEVAL-STLKT01V1: CleanALLMEMS1_SW4STM32_ST.bat	
   - Otherwise (Linux/iOS or Windows without the STM32 ST-Link Utility):
		- For STEVAL-STLKT01V1: CleanALLMEMS1_SW4STM32_ST.sh
    
    `cd ./STM32CubeFunctionPack_ALLMEMS1_V3.4.0/Projects/Multi/Applications/ALLMEMS1/SW4STM32/STM32L476RG-SensorTile`
    
    `chmod +x CleanALLMEMS1_SW4STM32_ST.sh`
    
    `./CleanALLMEMS1_SW4STM32_ST.sh`
    
## Bluepy Python

This firmware was used with some [Python scripts](https://github.com/MatteoOrlandini/Bluepy-Python-Thesis) that used the Bluepy library to scan and connect to the SensorTile device and enable BLE notification on Linux.

## Performance analysis
### Complementary filter

The figure below shows a comparison between the filtered (with a complementary filter) pitch data, in blue, and the data simply obtained from the formulas in which are used the accelerometer axis values, in red.

![](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/blob/master/Images/Pitch.png)

The figure below shows a comparison between the filtered (with a complementary filter) roll data, in blue, and the data simply obtained from the formulas in which are used the accelerometer axis values, in red.

![](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/blob/master/Images/Roll.png)
    
### Power consumption

The INA226 board was used to measure the current consumption, which allows to measure the current through a shunt resistance of 0.1 mΩ.

* When the microcontroller is in sleep the average current is 370.1 µA. To make this measurement, the state of the micro controller was forced in sleep using the MCU_PowerSave() function.

![](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/blob/master/Images/MCUoff.png)

* The average current during advertising is 12.2 mA. In the first 5 seconds the device is connected to the server, in the following ones the advertising data are sent.

![](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/blob/master/Images/connesso%2Badvertising.png)

You may notice current peaks every 50 ms, in fact the advertising data are transmitted at a frequency of 20 Hz.

![](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/blob/master/Images/connesso%2Badvertising_zoom.png)

* The average current while the device is connected and is streaming data is 15.1 mA.

![](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/blob/master/Images/Streaming.png)

You may notice current peaks every 10 ms, this is due to the transmission of data via Bluetooth at the frequency of 100 Hz.

![](https://github.com/MatteoOrlandini/SensorTile-Firmware-Thesis/blob/master/Images/Streaming_zoom.png)
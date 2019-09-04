/**
  ******************************************************************************
  * @file    main.h 
  * @author  Central LAB
  * @version V3.4.0
  * @date    26-Apr-2018
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "console.h" 

#include "osal.h"
#include "debug.h"
#include "ALLMEMS1_config.h"

/* Code for MotionFX integration - Start Section */
#include "MotionFX_Manager.h"
#include "motion_fx.h"
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
#include "MotionAR_Manager.h"
#include "motion_ar.h"
/* Code for MotionAR integration - End Section */
    
#ifdef ALLMEMS1_MOTIONCP
#include "MotionCP_Manager.h"
#include "motion_cp.h"
#endif /* ALLMEMS1_MOTIONCP */

#ifdef ALLMEMS1_MOTIONGR
#include "MotionGR_Manager.h"
#include "motion_gr.h"
#endif /* ALLMEMS1_MOTIONGR */

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
#include "AcousticSL_Manager.h"
#include "acoustic_sl.h"
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

extern unsigned char ReCallCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
extern unsigned char SaveCalibrationToMemory(uint16_t dataSize, uint32_t *data);

#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
    extern void RTC_DataConfig(uint8_t WeekDay, uint8_t Date, uint8_t Month, uint8_t Year);
    extern void RTC_TimeConfig(uint8_t Hours, uint8_t Minutes, uint8_t Seconds);
    extern void RTC_AlarmConfig(uint8_t StepHour, uint8_t StepMin, uint8_t StepSec);

    extern void RTC_GetCurrentDateTime(void);
  #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
#endif /* STM32_SENSORTILE */

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

/* Exported defines and variables  ------------------------------------------------------- */

/* Code for MotionFX integration - Start Section */
/* 10kHz/100 For MotionFX@100Hz as defaul value */
#define DEFAULT_uhCCR1_Val  100
/* Code for MotionFX integration - End Section */

#if (defined(ALLMEMS1_MOTIONCP) || defined(ALLMEMS1_MOTIONGR))
/* 10kHz/50 For MotionCP@50Hz or MotionGR@50Hz as defaul value */
#define DEFAULT_uhCCR2_Val  200
#endif /* defined(ALLMEMS1_MOTIONCP) || defined(ALLMEMS1_MOTIONGR) */

/* Code for MotionAR integration - Start Section */
/* 10kHz/16  For MotionAR@16Hz as defaul value */
#define DEFAULT_uhCCR3_Val  625
/* Code for MotionAR integration - End Section */

//10kHz/20  For Acc/Gyro/Mag@20Hz
//#define DEFAULT_uhCCR4_Val  500
//10kHz/100  For Acc/Gyro/Mag@100Hz
#define DEFAULT_uhCCR4_Val  100
//10kHz/100  For Acc/Gyro/Mag@1KHz
//#define DEFAULT_uhCCR4_Val  10

#ifdef STM32_BLUECOIN
  #define MIN_BATTERY_RANGE 2950
  #define MAX_BATTERY_RANGE 4225
  #define WINDOW_FILTER_VOLTAGE_VALUE_DIM 20
#endif /* STM32_BLUECOIN */

#ifndef STM32_NUCLEO
  #define BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER ((uint32_t)0x12345678)
#endif /* STM32_NUCLEO */

#ifdef STM32_SENSORTILE
  /* Range time without connect before Shut Down Mode starts (20 sec = 20 000 millisec) */
  //#define RANGE_TIME_WITHOUT_CONNECTED  20000
/* Range time without connect before Shut Down Mode starts (20 min = 1 200 sec = 1 200 000 millisec) */
	#define RANGE_TIME_WITHOUT_CONNECTED  1200000
#endif /* STM32_SENSORTILE */

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

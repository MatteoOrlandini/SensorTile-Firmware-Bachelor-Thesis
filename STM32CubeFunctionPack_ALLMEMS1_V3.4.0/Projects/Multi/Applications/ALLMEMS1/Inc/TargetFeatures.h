/**
  ******************************************************************************
  * @file    TargetFeatures.h 
  * @author  Central LAB
  * @version V3.4.0
  * @date    26-Apr-2018
  * @brief   Specification of the HW Features for each target platform
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
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
   
#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_hal.h"
  #ifdef STM32_NUCLEO
    #include "stm32f4xx_nucleo.h"
    #include "stm32f4xx_nucleo_bluenrg.h"
    #include "stm32f4xx_hal_conf.h"
    #include "stm32f4xx_UART.h"
    #include "stm32f4xx_I2C.h"
    #include "stm32f4xx_SPI.h"
    #include "stm32f4xx_periph_conf.h"
    #include "stm32_bluenrg_ble.h"
    #ifdef IKS01A1   
      #include "x_nucleo_iks01a1.h"
      #include "x_nucleo_iks01a1_accelero.h"
      #include "x_nucleo_iks01a1_gyro.h"
      #include "x_nucleo_iks01a1_magneto.h"
      #include "x_nucleo_iks01a1_humidity.h"
      #include "x_nucleo_iks01a1_temperature.h"
      #include "x_nucleo_iks01a1_pressure.h"
    #elif IKS01A2
      #include "x_nucleo_iks01a2.h"
      #include "x_nucleo_iks01a2_accelero.h"
      #include "x_nucleo_iks01a2_gyro.h"
      #include "x_nucleo_iks01a2_magneto.h"
      #include "x_nucleo_iks01a2_humidity.h"
      #include "x_nucleo_iks01a2_temperature.h"
      #include "x_nucleo_iks01a2_pressure.h"
    #endif /* IKS01A1 */
    #include "x_nucleo_cca02m1_audio_f4.h"      
  #elif STM32_BLUECOIN
    #include "BlueCoin.h"
    #include "BlueCoin_BlueNRG.h"
    #include "BlueCoin_accelero.h"
    #include "BlueCoin_gyro.h"
    #include "BlueCoin_pressure.h"
    #include "BlueCoin_magneto.h"
    #include "BlueCoin_audio_in.h"
    //#include "BlueCoin_audio_out.h"
    #include "BlueCoin_temperature.h"
    #include "BlueCoin_sd.h"
    #endif /* STM32_NUCLEO */
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#include "stm32l4xx_hal.h"
  #ifdef STM32_NUCLEO
    #include "stm32l4xx_nucleo.h"
    #include "stm32l4xx_nucleo_bluenrg.h"
    #include "stm32l4xx_hal_conf.h"
    #include "stm32l4xx_UART.h"
    #include "stm32l4xx_I2C.h"
    #include "stm32l4xx_SPI.h"
    #include "stm32l4xx_periph_conf.h"
    #include "stm32_bluenrg_ble.h"
    #ifdef IKS01A1   
      #include "x_nucleo_iks01a1.h"
      #include "x_nucleo_iks01a1_accelero.h"
      #include "x_nucleo_iks01a1_gyro.h"
      #include "x_nucleo_iks01a1_magneto.h"
      #include "x_nucleo_iks01a1_humidity.h"
      #include "x_nucleo_iks01a1_temperature.h"
      #include "x_nucleo_iks01a1_pressure.h"
    #elif IKS01A2
      #include "x_nucleo_iks01a2.h"
      #include "x_nucleo_iks01a2_accelero.h"
      #include "x_nucleo_iks01a2_gyro.h"
      #include "x_nucleo_iks01a2_magneto.h"
      #include "x_nucleo_iks01a2_humidity.h"
      #include "x_nucleo_iks01a2_temperature.h"
      #include "x_nucleo_iks01a2_pressure.h"
    #endif /* IKS01A1 */
    #include "x_nucleo_cca02m1_audio_l4.h"
  #elif STM32_SENSORTILE
    #include "SensorTile.h"
    #include "stm32l4xx_hal_conf.h"
    #include "stm32l4xx_hal_def.h"
    #include "SensorTile_BlueNRG.h"
    #include "SensorTile_accelero.h"
    #include "SensorTile_gyro.h"
    #include "SensorTile_magneto.h"
    #include "SensorTile_pressure.h"
    #include "SensorTile_temperature.h"
    #include "SensorTile_humidity.h"
    #include "SensorTile_gg.h"
    #include "SensorTile_audio_in.h"
    #include "SensorTile_sd.h"
#endif /* STM32_NUCLEO */
#endif /* USE_STM32L4XX_NUCLEO */
   
#include "ALLMEMS1_config.h"
#include "MetaDataManager.h"

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

/* Code for BlueVoice integration - Start Section */
#include "AudioBV_Manager.h"
#include "bluevoice_adpcm.h"
/* Code for BlueVoice integration - End Section */

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
#include "AcousticSL_Manager.h"
#include "acoustic_sl.h"
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */
   
#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
#include "AcousticBF_Manager.h"
#include "acoustic_bf.h"
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */

/* Exported defines ------------------------------------------------------- */
#define MAX_TEMP_SENSORS 2

/* BlueNRG Board Type */
#define IDB04A1 0
#define IDB05A1 1
   
/* Mems Board Type */
#define _IKS01A1 0
#define _IKS01A2 1

/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */
/**
 * @brief  Target type data definition
 */
typedef enum
{
  TARGET_NUCLEO,
  TARGET_BLUECOIN,
  TARGET_SENSORTILE,
  TARGETS_NUMBER
} TargetType_t;

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  TargetType_t BoardType;
  int32_t NumTempSensors;
  void *HandleTempSensors[MAX_TEMP_SENSORS];

  void *HandlePressSensor;
  void *HandleHumSensor;

  int32_t HWAdvanceFeatures;
  void *HandleAccSensor;
  void *HandleGyroSensor;
  void *HandleMagSensor;
  
  int32_t NumMicSensors;

  uint8_t LedStatus;
  uint8_t bnrg_expansion_board;
  uint8_t mems_expansion_board;
    
#ifdef STM32_SENSORTILE
  void *HandleGGComponent;
#endif /* STM32_SENSORTILE */

  /* Code for MotionFX integration - Start Section */
  uint32_t MotionFXIsInitalized;
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  uint32_t MotionARIsInitalized;
  /* Code for MotionAR integration - End Section */

  #ifdef ALLMEMS1_MOTIONCP
  uint32_t MotionCPIsInitalized;
  #endif /* ALLMEMS1_MOTIONCP */
  
  #ifdef ALLMEMS1_MOTIONGR
  uint32_t MotionGRIsInitalized;
  #endif /* ALLMEMS1_MOTIONGR */
  
  /* Code for BlueVoice integration - Start Section */
  uint32_t AudioBVIsInitalized;
  /* Code for BlueVoice integration - End Section */
  
  /* Acoustic Source Localization Library */
#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
  uint32_t AcousticSLIsInitalized;
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */
 /* Acoustic Beam Forming Recognition Library */
#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
  uint32_t AcousticBFIsInitalized;
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

#ifdef STM32_BLUECOIN
extern void PVD_Config(void);
#endif /* STM32_BLUECOIN */

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(TargetType_t BoardType);

extern void InitMics(uint32_t AudioFreq, uint32_t AudioVolume);
extern void DeInitMics(void);

extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);
extern void LedToggleTargetPlatform(void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/


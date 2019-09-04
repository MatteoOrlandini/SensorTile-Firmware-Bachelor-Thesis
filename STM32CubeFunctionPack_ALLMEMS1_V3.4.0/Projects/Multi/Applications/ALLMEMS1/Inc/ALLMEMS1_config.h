/**
  ******************************************************************************
  * @file    ALLMEMS1_config.h
  * @author  Central LAB
  * @version V3.4.0
  * @date    26-Apr-2018
  * @brief   FP-SNS-ALLMEMS1 configuration
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
#ifndef __ALLMEMS1_CONFIG_H
#define __ALLMEMS1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* For enabling SD card recording */
//The SD card data logging is enabled by default.
//disable this feature for the STEVAL-STLKT01V1.
//#ifdef  STM32_SENSORTILE
//#define ALLMEMS1_ENABLE_SD_CARD_LOGGING
//#endif /* STM32_SENSORTILE */

/* For enabling MotionCP integration */
#ifdef USE_STM32L4XX_NUCLEO
#define ALLMEMS1_MOTIONCP
#endif /* USE_STM32L4XX_NUCLEO */

/* For enabling MotionGR integration */
#ifdef USE_STM32L4XX_NUCLEO
#define ALLMEMS1_MOTIONGR
#endif /* USE_STM32L4XX_NUCLEO */

/* For enabling AcousticSL integration */
#ifndef STM32_SENSORTILE
  #ifndef ALLMEMS1_ENABLE_SD_CARD_LOGGING
    #define ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
  #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
#endif /* STM32_SENSORTILE */


/* For enabling AcousticBF integration */
#ifdef USE_STM32F4XX_NUCLEO
#define ALLMEMS1_ACOUSTIC_BEAM_FORMING
#endif /* USE_STM32F4XX_NUCLEO */

/* Define the ALLMEMS1 MAC address, otherwise it will create a Unique MAC */
//#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

#ifndef MAC_BLUEMS
/* For creating one MAC related to STM32 UID, Otherwise the BLE will use it's random MAC */
#define MAC_STM32UID_BLUEMS
#endif /* MAC_BLUEMS */

/* Define The transmission interval in Multiple of 10ms for quaternions*/
#define QUAT_UPDATE_MUL_10MS 3

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3

/* IMPORTANT 
The Sensors fusion runs at 100Hz so like MAXIMUM it possible to send:
1 quaternion every 10ms
2 quaternions every 20ms
3 quaternions every 30ms

if QUAT_UPDATE_MUL_10MS!=3, then SEND_N_QUATERNIONS must be ==1
*/

/*************** Debug Defines ******************/
#ifndef STM32_BLUECOIN
/* For enabling the printf on UART */
#ifdef STM32_SENSORTILE
  /* Enabling this define for SensorTile..
   * it will introduce a delay of 10Seconds before starting the application
   * for having time to open the Terminal
   * for looking the ALLMEMS1 Initialization phase */

//DISABLED BY DEFAULT
//  #ifndef ALLMEMS1_ENABLE_SD_CARD_LOGGING
//    #define ALLMEMS1_ENABLE_PRINTF
//  #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
#else /* STM32_SENSORTILE */
  /* For Nucleo it's enable by default */
  #define ALLMEMS1_ENABLE_PRINTF
#endif /* STM32_SENSORTILE */
#endif /* STM32_BLUECOIN */

/* For enabling connection and notification subscriptions debug */
#define ALLMEMS1_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define ALLMEMS1_DEBUG_NOTIFY_TRAMISSION

/* Define The transmission interval in Multiple of 10ms for Microphones dB Values */
#define MICS_DB_UPDATE_MUL_10MS 5

/* Define The transmission interval in Multiple of 10ms for Microphones dB Values */
#define ENV_UPDATE_MUL_100MS 5

/* For enabling the License Parsing Debug */
#define ALLMEMS1_LIC_PARSING


/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define ALLMEMS1_VERSION_MAJOR '3'
#define ALLMEMS1_VERSION_MINOR '4'
#define ALLMEMS1_VERSION_PATCH '0'

/* Define the ALLMEMS1 Name MUST be 7 char long */
#define NAME_BLUEMS 'A','M','1','V',ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH

/* Package Name */
#define ALLMEMS1_PACKAGENAME "FP-SNS-ALLMEMS1"

#ifdef STM32_NUCLEO
  #define AUDIO_VOLUME_VALUE       64
  #define AUDIO_CHANNELS           2
  #define AUDIO_SAMPLING_FREQUENCY 16000
  #define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000

  /* Code for Acoustic Source Localization integration - Start Section */  
  #define M1_EXT_B 0
  #define M4_EXT_B 1
     
  /* Microphone distance */
  #define SIDE     147
  /* Code for Acoustic Source Localization integration - End Section */   
#endif /* STM32_NUCLEO */

#ifdef STM32_SENSORTILE
  #define AUDIO_VOLUME_VALUE       4
  #define AUDIO_CHANNELS           1
  #define AUDIO_SAMPLING_FREQUENCY 16000
  #define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000
     

  #ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
    /* Defines related to Clock configuration */
    /* Uncomment to enable the adaquate Clock Source */
    /* #define RTC_CLOCK_SOURCE_LSI */
    #define RTC_CLOCK_SOURCE_LSE

    #ifdef RTC_CLOCK_SOURCE_LSI
      #define RTC_ASYNCH_PREDIV    0x7F
      #define RTC_SYNCH_PREDIV     0xF9
    #endif /* RTC_CLOCK_SOURCE_LSI */

    #ifdef RTC_CLOCK_SOURCE_LSE
      #define RTC_ASYNCH_PREDIV  0x7F
      #define RTC_SYNCH_PREDIV   0x00FF
    #endif /* RTC_CLOCK_SOURCE_LSE */
  #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

#endif /* STM32_SENSORTILE */

#ifdef STM32_BLUECOIN
  #define AUDIO_VOLUME_VALUE       64
  #define AUDIO_CHANNELS           4
  #define AUDIO_SAMPLING_FREQUENCY 16000
  #define PCM_AUDIO_IN_SAMPLES     AUDIO_SAMPLING_FREQUENCY/1000

  /* Code for Acoustic Source Localization integration - Start Section */ 
  #define M1_MIC 0
  #define M2_MIC 1
  #define M3_MIC 2
  #define M4_MIC 3
     
  #define TOP_RIGHT_MIC         M3_MIC
  #define TOP_LEFT_MIC          M4_MIC
  #define BOTTOM_RIGHT_MIC      M1_MIC
  #define BOTTOM_LEFT_MIC       M2_MIC
    
  /* Microphone distance */
  #define DIAGONAL        57
  #define SIDE            40
  /* Code for Acoustic Source Localization integration - End Section */
#endif /* STM32_BLUECOIN */
     
/* Code for BlueVoice integration - Start Section */
#define BV_AUDIO_SAMPLING_FREQUENCY     8000
#define BV_AUDIO_VOLUME_VALUE           64
#define PCM_IN_SAMPLES_PER_MS           (((uint16_t)BV_AUDIO_SAMPLING_FREQUENCY)/1000)
#define AUDIO_IN_MS                     (1)       /*!< Number of ms of Audio given as input to the BlueVoice library.*/ 
#define BV_PCM_AUDIO_IN_SAMPLES         (PCM_IN_SAMPLES_PER_MS * AUDIO_IN_MS)
/* Code for BlueVoice integration - End Section */

#ifdef ALLMEMS1_ENABLE_PRINTF
  #ifdef STM32_NUCLEO
    #define ALLMEMS1_PRINTF(...) printf(__VA_ARGS__)
  #elif STM32_SENSORTILE
    #include "usbd_cdc_interface.h"
    #define ALLMEMS1_PRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }
  #endif /* STM32_NUCLEO */
#else /* ALLMEMS1_ENABLE_PRINTF */
  #define ALLMEMS1_PRINTF(...)
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)
/* Control Section */

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
  #error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
  #error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif

#endif /* __ALLMEMS1_CONFIG_H */

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

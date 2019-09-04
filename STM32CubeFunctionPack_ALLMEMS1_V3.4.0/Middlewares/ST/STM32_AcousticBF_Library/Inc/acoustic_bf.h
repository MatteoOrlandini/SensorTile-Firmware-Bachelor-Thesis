/**
******************************************************************************
* @file    acoustic_bf.h
* @author  Central Labs
* @version V2.1.1
* @date    15-February-2018
* @brief   This file contains Acoustic Beamforming library definitions.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*
* Licensed under Software License Agreement SLA0077, (the "License"). 
* You may not use this package except in compliance with the License. 
* You may obtain a copy of the License at:
*
* http://www.st.com/content/st_com/en/search.html#q=SLA0077-t=keywords-page=1
*
* Some of the library code is based on the open Source patent-free SPEEX software,
* by Jean-Marc Valin/Xiph.Org Foundation, released under "revised BSD" licence.
* Licencing terms are available in the attached release_note.html file, in the
* libSpeexAEC100 application note, in the next lines of this document and it's
* available on the web at: http://www.xiph.org/licenses/bsd/speex/
*
*   SPEEX revised BSD licence note:
*
* Â© 2002-2003, Jean-Marc Valin/Xiph.Org Foundation.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
* - Neither the name of the Xiph.org Foundation nor the names of its contributors
*   may be used to endorse or promote products derived from this software without
*   specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Some of the library code is based on the CMSIS DSP software library by ARM,
* a suite of common signal processing functions for use on Cortex-M processor
* based devices. Licencing terms are available in the attached release_note.html
* file, in the libSoundSourceLoc100 application note, in the next lines of this
* document and it's available on the web at:
* http://www.keil.com/pack/doc/CMSIS/DSP/html/index.html
*
*   ARM licence note:
*
* Copyright (C) 2009-2012 ARM Limited.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*  - Neither the name of ARM nor the names of its contributors may be used
*   to endorse or promote products derived from this software without
*   specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACOUSTIC_BF_H
#define __ACOUSTIC_BF_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/** @addtogroup MIDDLEWARES
* @{
*/

/** @defgroup ACOUSTIC_BF ACOUSTIC_BF
* @{
*/

/** @defgroup ACOUSTIC_BF_Exported_Constants AcousticBF Exported Constants
* @{
*/  

/** @defgroup ACOUSTIC_BF_algorithm_type
* @brief    Beam Forming algorithm type 
* @{
*/ 
#define ACOUSTIC_BF_TYPE_CARDIOID_BASIC              	((uint32_t)0x00000000)
#define ACOUSTIC_BF_TYPE_CARDIOID_DENOISE            	((uint32_t)0x00000001)
#define ACOUSTIC_BF_TYPE_ASR_READY                   	((uint32_t)0x00000002)
#define ACOUSTIC_BF_TYPE_STRONG                      	((uint32_t)0x00000003)
/**
* @}
*/

/** @defgroup ACOUSTIC_BF_data_format
* @brief    Beam Forming data format 
* @{
*/ 
#define ACOUSTIC_BF_DATA_FORMAT_PDM                  	((uint32_t)0x00000000)
#define ACOUSTIC_BF_DATA_FORMAT_PCM                  	((uint32_t)0x00000001)
/**
* @}
*/
  /** @defgroup ACOUSTIC_BF_reference_channel
* @brief    Beam Forming reference channel 
* @{
*/ 
#define ACOUSTIC_BF_REF_DISABLE                      	((uint32_t)0x00000000)
#define ACOUSTIC_BF_REF_ENABLE                       	((uint32_t)0x00000001)
/**
* @}
*/

  /** @defgroup ACOUSTIC_BF_delay
* @brief    Beam Forming delay 
* @{
*/ 
#define ACOUSTIC_BF_DELAY_DISABLE                      	((uint32_t)0x00000000)
#define ACOUSTIC_BF_DELAY_ENABLE                       	((uint32_t)0x00000001)
/**
* @}
*/

  /** @defgroup ACOUSTIC_BF_sampling_frequency
* @brief    Beam Forming sampling frequency 
* @{
*/ 
#define ACOUSTIC_BF_FS_16                             	((uint32_t)16)
#define ACOUSTIC_BF_FS_256                            	((uint32_t)256)
#define ACOUSTIC_BF_FS_384                            	((uint32_t)384)
#define ACOUSTIC_BF_FS_512                            	((uint32_t)512)
#define ACOUSTIC_BF_FS_1024                           	((uint32_t)1024)
#define ACOUSTIC_BF_FS_1280                           	((uint32_t)1280)
#define ACOUSTIC_BF_FS_2048                           	((uint32_t)2048)
#define ACOUSTIC_BF_FS_3072                           	((uint32_t)3072)

/**
* @}
*/

  /** @defgroup ACOUSTIC_BF_errors
* @brief    Beam Forming errors 
* @{
*/ 
#define ACOUSTIC_BF_TYPE_ERROR                       	((uint32_t)0x00000001)
#define ACOUSTIC_BF_PTR_CHANNELS_ERROR               	((uint32_t)0x00000002)
#define ACOUSTIC_BF_DATA_FORMAT_ERROR                	((uint32_t)0x00000004)
#define ACOUSTIC_BF_SAMPLING_FREQ_ERROR              	((uint32_t)0x00000008)
#define ACOUSTIC_BF_M2_GAIN_ERROR                    	((uint32_t)0x00000010)
#define ACOUSTIC_BF_DISTANCE_ERROR                   	((uint32_t)0x00000020)
#define ACOUSTIC_BF_REF_OUT_ERROR                    	((uint32_t)0x00000040)
#define ACOUSTIC_BF_DELAY_ERROR                      	((uint32_t)0x00000080)
#define ACOUSTIC_BF_PROCESSING_ERROR                 	((uint32_t)0x00000100)


#ifndef ACOUSTIC_LOCK_ERROR
#define ACOUSTIC_LOCK_ERROR                         	((uint32_t)0x10000000)
#endif 
/**
* @}
*/
/**
* @}
*/
  
/** @defgroup ACOUSTIC_BF_Exported_Types AcousticBF Exported Types
* @{
*/
/**
 * @brief  Library handler. It keeps track of the static parameters
 *         and it handles the internal state of the algorithm.
 */
typedef struct
{
  uint32_t data_format;                         /*!< Specifies the data format for input: PDM or PCM. This parameter can be a value of @ref ACOUSTIC_BF_data_format. Default value is ACOUSTIC_BF_DATA_FORMAT_PDM */
  uint32_t sampling_frequency;                  /*!< Specifies the sampling frequency in KHz - can be 16 Hz for PCM,
                                                     1024 for PDM. This parameter can be a value of @ref ACOUSTIC_BF_data_format. Default value is ACOUSTIC_BF_DATA_FORMAT_PDM */
  uint8_t ptr_M1_channels;                      /*!< Number of channels in the stream of Microphone 1. Can be any integer > 0. Defualt value is 2*/
  uint8_t ptr_M2_channels;                      /*!< Number of channels in the stream of Microphone 2. Can be any integer > 0. Defualt value is 2 */
  uint8_t ptr_out_channels;                     /*!< Number of channels in the output stream. Can be any integer > 0. Defualt value is 2 */
  uint8_t algorithm_type_init;                  /*!< Specifies the type of algorithm  used in the initialization function.
                                                     On this parameter depends the amount of memory required.  This parameter can be a value of @ref ACOUSTIC_BF_algorithm_type. Default value is ACOUSTIC_BF_TYPE_CARDIOID_BASIC */
  uint32_t ref_mic_enable;                      /*!< Enable or disable the omnidirectional microphone reference
                                                     in the output stream. This parameter can be a value of @ref ACOUSTIC_BF_reference_channel. Default value is ACOUSTIC_BF_REF_ENABLE*/
  uint8_t delay_enable;                         /*! Enable the delay performed inside the library. If delay is performed outside the library, PCM input must be chosen*/
  uint32_t internal_memory_size;                /*!< Keeps track of the amount of memory required for the current setup.
                                                     It's filled by the Beamforming_getMemorySize() function and must be
                                                     used to allocate the right amount of RAM */
  uint32_t * pInternalMemory;                   /*!< Pointer to the internal algorithm memory */
  
} AcousticBF_Handler_t;

/**
 * @brief  Library dynamic configuration handler. It contains dynamic parameters.
 */
typedef struct
{
  uint16_t mic_distance;                        /*!< Distance between Mic1 and Mic2. It must be specified in tenths of a
                                                     millimeter. For example, if the microphone distance is equal to 4 mm,
                                                     this parameter must be initialized with the value 40. Default value is 150. */
  uint32_t algorithm_type;                      /*!< Type of algorithm to switch to. Switching from one algorithm type to
                                                     another depends on the the type of initialization used. If the library
                                                     has been initialized using the STRONG option, switching to any other
                                                     algorithm is possible; if it's initialized as ASR_READY or LIGHT the
                                                     switch is possible only towards BASIC_CARDIOID, if it has been
                                                     initialized as Basic cardioid, switching is not admitted. This is due
                                                     to the different amount of data structures initialized depending on the
                                                     chosen algorithm and to the decision to allocate the minimum memory
                                                     amount needed. This parameter can be a value of @ref ACOUSTIC_BF_algorithm_type. 
                                                     Default value is ACOUSTIC_BF_TYPE_CARDIOID_BASIC */
  int8_t volume;                               /*!< Overall gain of the algorithm. It specifies the amound of gain added to the microphones, in dB. 
													It's used only when PDM input is chosen.*/
  float M2_gain;                                /*!< Gain to be applied to the second microphone respect to the first one.
                                                     If set to 0, automatic gain is used */
} 
AcousticBF_Config_t;

/**
  * @}
  */
  
  /** @defgroup ACOUSTIC_BF_Exported_Functions AcousticBF Exported Functions
 * @{
 */

/**
 * @brief  Fills the "internal_memory_size" of the pHandler parameter passed as argument with a value representing the
 *         right amount of memory needed by the library, depending on the specific static parameters adopted.
 * @param  pHandler: libBeamforming_Handler filled with desired parameters.
 * @retval 0 if everything is fine.
 */
uint32_t AcousticBF_getMemorySize(AcousticBF_Handler_t * pHandler);

/**
 * @brief  Library initialization
 * @param  pHandler: AcousticBF_Handler_t filled with desired parameters.
 * @retval 0 if everything is fine.
 *         different from 0 if erroneous parameters have been passed to the Init function and the default value has been used.
 *         The specific error can be recognized by checking the relative bit in the returned word.
 */
uint32_t AcousticBF_Init(AcousticBF_Handler_t * pHandler);

/**
 * @brief  Library data input/output
 * @param  pM1: pointer to an array that contains PCM or PDM samples of the first channel (1 millisecond).
 * @param  pM2: pointer to an array that contains PCM or PDM samples of the second channel (1 millisecond).
 * @param  ptr_Out: pointer to an array that will contain PCM samples of output data (1 millisecond). If the reference channel
 *         is activated, two channels will be written in the output array.
 * @param  pHandler: pointer to the handler of the current Beamforming instance running.
 * @retval 1 if data collection is finished and Beamforming_SecondStep must be called, 0 otherwise.
 * @note   Input/output function reads and write samples skipping the required number of values depending on the
 *         ptr_Mx_channels configuration.
 */
uint32_t AcousticBF_FirstStep(void *pM1, void *pM2, void *ptr_Out, AcousticBF_Handler_t * pHandler);

/**
 * @brief  Library run function, performs audio analysis when all required data has been collected.
 * @param  pHandler: pointer to the handler of the current beamforming instance running.
 * @retval 0 if everything is ok.
 */
uint32_t AcousticBF_SecondStep(AcousticBF_Handler_t * pHandler);

/**
 * @brief  Library setup function, it sets the values for dynamic parameters. It can be called at runtime to change
 *         dynamic parameters.
 * @param  pHandler: pointer to the handler of the current Beamforming instance running.
 * @param  pConfig: pointer to the dynamic parameters handler containing the new library configuration.
 * @retval 0 if everything is fine.
 *         different from 0 if erroneous parameters have been passed to the setConfig function and the default
 *         value has been used. The specific error can be recognized by checking the relative bit in the returned word.
 */
uint32_t AcousticBF_setConfig(AcousticBF_Handler_t * pHandler, AcousticBF_Config_t * pConfig);

/**
 * @brief  Fills the pConfig structure with the actual dynamic parameters as they are used inside the library.
 * @param  pHandler: pointer to the handler of the current Beamforming instance running.
 * @param  pConfig: pointer to the dynamic parameters handler that will be filled with the current library configuration.
 * @retval 0 if everything is fine.
 */
uint32_t AcousticBF_getConfig(AcousticBF_Handler_t * pHandler, AcousticBF_Config_t * pConfig);

/**
 * @brief  To be used to retrieve version information.
 * @param  version char array to be filled with the current library version
 * @retval 0 if everything is fine.
*/
uint32_t AcousticBF_GetLibVersion(char *version);


/**
  * @}
  */
  
  /**
  * @}
  */

/**
  * @}
  */
#endif  /*__ACOUSTIC_BF_H*/

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/

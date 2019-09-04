 /**
 ******************************************************************************
 * @file    AcousticBF_Manager.h
 * @author  Central Lab
 * @version V3.4.0
 * @date    26-Apr-2018
 * @brief   Header for AcousticBF_Manager.c
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
#ifndef _ACOUSTICBF_MANAGER_H_
#define _ACOUSTICBF_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Define Acoustic Beam Forming for static parameters */
#ifdef STM32_BLUECOIN
  #define AUDIO_SAMPLING_FREQUENCY_BF   2048
#elif STM32_NUCLEO
//  #ifdef STM32F401xE
    #define AUDIO_SAMPLING_FREQUENCY_BF 1280
//  #elif STM32F446xx
//    #define AUDIO_SAMPLING_FREQUENCY_BF 1024
//  #endif /* STM32F401xE */
#endif /* STM32_BLUECOIN */
  
#define OUTPUT_STREAM_NUMBER_CHANNELS   2
  
/* Define Acoustic Beam Forming for for dynamic parameters */
#define GAIN_APPLIED_TO_SECOND_MICROPHONE       (0.0f)
#define AUDIO_BF_VOLUME_VALUE                   20

#define  RANGE_INF_MIC_M4_EXT_B 0
#define  RANGE_SUP_MIC_M4_EXT_B 80
 
#define  RANGE_INF_MIC_M1_EXT_B 100
#define  RANGE_SUP_MIC_M1_EXT_B 180



/* Exported Functions Prototypes ---------------------------------------------*/
extern void AcousticBF_Manager_init(void);
extern void SW_Task1_Callback(void);
extern void AudioProcess_BF(int32_t dir);
extern void BeamFormingSetType(uint32_t type);

extern void BeamFormingDirectionSetup(int32_t Direction);

#ifdef __cplusplus
}
#endif

#endif //_ACOUSTICBF_MANAGER_H_

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


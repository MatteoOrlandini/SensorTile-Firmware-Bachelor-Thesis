/**
******************************************************************************
* @file    BlueCoin_audio_out.h
* @author  Central Labs
* @version V1.2.0
* @date    23-March-2018
* @brief   This file contains definitions for BlueCoin_audio_out.c driver.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#ifndef __BLUECOIN_AUDIO_OUT_H
#define __BLUECOIN_AUDIO_OUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../Components/pcm1774/PCM1774_CODEC_driver.h"
#include "BlueCoin.h"

/** @addtogroup BSP BSP
* @{
*/

/** @addtogroup BlueCoin BlueCoin
* @{
*/

/** @addtogroup BlueCoin_Audio_OUT 
* @{
*/




/** @addtogroup BlueCoin_Audio_OUT_Exported_Types 
* @{
*/
typedef enum
{
  CODEX_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  PCM1774_0 = 0                   /* . */
} CODEX_ID_t;
/**
* @}
*/

/** @addtogroup BlueCoin_Audio_OUT_Exported_Variables  
* @{
*/
extern DMA_HandleTypeDef hdma_dfsdmReg_FLT[];
extern SAI_HandleTypeDef haudio_out_sai;
/**
* @}
*/


/** @addtogroup BlueCoin_Audio_OUT_Exported_Constants 
* @{
*/

#define CODEC_SENSORS_MAX_NUM 2



#ifndef AUDIO_FREQUENCY_192K
#define AUDIO_FREQUENCY_192K          ((uint32_t)192000)
#endif
#ifndef AUDIO_FREQUENCY_96K
#define AUDIO_FREQUENCY_96K           ((uint32_t)96000)
#endif
#ifndef AUDIO_FREQUENCY_48K
#define AUDIO_FREQUENCY_48K           ((uint32_t)48000)
#endif
#ifndef AUDIO_FREQUENCY_44K
#define AUDIO_FREQUENCY_44K           ((uint32_t)44100)
#endif
#ifndef AUDIO_FREQUENCY_32K
#define AUDIO_FREQUENCY_32K           ((uint32_t)32000)
#endif
#ifndef AUDIO_FREQUENCY_22K
#define AUDIO_FREQUENCY_22K           ((uint32_t)22050)
#endif
#ifndef AUDIO_FREQUENCY_16K
#define AUDIO_FREQUENCY_16K           ((uint32_t)16000)
#endif
#ifndef AUDIO_FREQUENCY_11K
#define AUDIO_FREQUENCY_11K           ((uint32_t)11025)
#endif
#ifndef AUDIO_FREQUENCY_8K
#define AUDIO_FREQUENCY_8K            ((uint32_t)8000)
#endif

#ifndef AUDIO_MUTE_ON
#define AUDIO_MUTE_ON            ((uint32_t)1)
#endif
#ifndef AUDIO_MUTE_OFF
#define AUDIO_MUTE_OFF            ((uint32_t)0)
#endif


/*------------------------------------------------------------------------------
                          USER SAI defines parameters
 -----------------------------------------------------------------------------*/
/** @defgroup CODEC_AudioFrame_SLOT_TDMMode
  * @brief In W8994 codec the Audio frame contains 4 slots : TDM Mode
  * TDM format :
  * +------------------|------------------|--------------------|-------------------+
  * | CODEC_SLOT0 Left | CODEC_SLOT1 Left | CODEC_SLOT0 Right  | CODEC_SLOT1 Right |
  * +------------------------------------------------------------------------------+
  * @{
  */
/* To have 2 separate audio stream in Both headphone and speaker the 4 slot must be activated */
#define CODEC_AUDIOFRAME_SLOT_0123                   SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_2 | SAI_SLOTACTIVE_3
/* To have an audio stream in headphone only SAI Slot 0 and Slot 2 must be activated */
#define CODEC_AUDIOFRAME_SLOT_02                     SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_2
/* To have an audio stream in speaker only SAI Slot 1 and Slot 3 must be activated */
#define CODEC_AUDIOFRAME_SLOT_13                     SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_3
/**
  * @}
  */

/* SAI peripheral configuration defines */
#define AUDIO_SAIx                           SAI1_Block_A
#define AUDIO_SAIx_CLK_ENABLE()              __SAI1_CLK_ENABLE()
#define AUDIO_SAIx_MCLK_SCK_SD_FS_AF         GPIO_AF6_SAI1

#define AUDIO_SAIx_SCK_ENABLE()              __GPIOB_CLK_ENABLE()
#define AUDIO_SAIx_SD_ENABLE()               __GPIOB_CLK_ENABLE()
#define AUDIO_SAIx_MCLK_ENABLE()             __GPIOE_CLK_ENABLE()
#define AUDIO_SAIx_FS_ENABLE()               __GPIOE_CLK_ENABLE()
#define AUDIO_SAIx_SCK_GPIO_PORT             GPIOB
#define AUDIO_SAIx_SD_GPIO_PORT              GPIOB
#define AUDIO_SAIx_MCLK_GPIO_PORT            GPIOE
#define AUDIO_SAIx_FS_GPIO_PORT              GPIOE
#define AUDIO_SAIx_SCK_PIN                   GPIO_PIN_10
#define AUDIO_SAIx_SD_PIN                    GPIO_PIN_2
#define AUDIO_SAIx_FS_PIN                    GPIO_PIN_4
#define AUDIO_SAIx_MCLK_PIN                  GPIO_PIN_2

/* SAI DMA Stream definitions */
#define AUDIO_SAIx_DMAx_CLK_ENABLE()         __DMA2_CLK_ENABLE()
#define AUDIO_SAIx_DMAx_STREAM               DMA2_Stream1
#define AUDIO_SAIx_DMAx_CHANNEL              DMA_CHANNEL_0
#define AUDIO_SAIx_DMAx_IRQ                  DMA2_Stream1_IRQn
#define AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_SAIx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SZE                          0xFFFF

#define AUDIO_SAIx_DMAx_IRQHandler           DMA2_Stream1_IRQHandler

/* Select the interrupt preemption priority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           5   /* Select the preemption priority level(0 is the highest) */

/* AUDIO FREQUENCY */
#define AUDIO_FREQUENCY_192K          ((uint32_t)192000)
#define AUDIO_FREQUENCY_96K           ((uint32_t)96000)
#define AUDIO_FREQUENCY_48K           ((uint32_t)48000)
#define AUDIO_FREQUENCY_44K           ((uint32_t)44100)
#define AUDIO_FREQUENCY_32K           ((uint32_t)32000)
#define AUDIO_FREQUENCY_22K           ((uint32_t)22050)
#define AUDIO_FREQUENCY_16K           ((uint32_t)16000)
#define AUDIO_FREQUENCY_11K           ((uint32_t)11025)
#define AUDIO_FREQUENCY_8K            ((uint32_t)8000)
/**
* @}
*/


/*------------------------------------------------------------------------------
             CONFIGURATION: Audio Driver Configuration parameters
------------------------------------------------------------------------------*/

/** @defgroup BlueCoin_Audio_OUT_Exported_Macros  BlueCoin_Audio_OUT_Exported Macros
* @{
*/
#define AUDIODATA_SIZE                      2   /* 16-bits audio data size */

/* Audio status definition */
#ifndef  AUDIO_OK
#define AUDIO_OK                        0
#endif
#ifndef  AUDIO_ERROR
#define AUDIO_ERROR                     1
#endif
#ifndef  AUDIO_TIMEOUT
#define AUDIO_TIMEOUT                   2
#endif

#define AUDIOFREQ_48K                ((uint32_t)48000)
#define AUDIOFREQ_32K                ((uint32_t)32000)
#define AUDIOFREQ_16K                ((uint32_t)16000)
#define AUDIOFREQ_8K                 ((uint32_t)8000)


/* Those defines are used to allocate the right amount of RAM depending on the
   maximum number of microphone and frequency desired */
#define MAX_AUDIO_IN_FREQ                 AUDIOFREQ_48K
#define MAX_AUDIO_IN_CHANNEL_NBR          4
#define DECIMATION_FACTOR                 64

/*BSP internal buffer size in half words (16 bits)*/
#ifndef PDM_INTERNAL_BUFFER_SIZE
#define PDM_INTERNAL_BUFFER_SIZE          MAX_AUDIO_IN_FREQ / 1000 * DECIMATION_FACTOR * 2 / 8
#endif

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)

/**
* @}
*/

/** @addtogroup BlueCoin_Audio_OUT_Exported_Functions  
* @{
*/
uint8_t BSP_AUDIO_OUT_Init(CODEX_ID_t id, void **handle, uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
uint8_t BSP_AUDIO_OUT_Play(void *handle, uint16_t* pBuffer, uint32_t Size);
uint8_t BSP_AUDIO_OUT_Pause(void *handle);
uint8_t BSP_AUDIO_OUT_Resume(void *handle);
uint8_t BSP_AUDIO_OUT_SetFrequency(void *handle, uint32_t AudioFreq);
static DrvStatusTypeDef BSP_PCM1774_CODEC_Init( void **handle, uint8_t Volume, uint32_t AudioFreq);

__weak uint8_t BSP_AUDIO_OUT_ClockConfig(uint32_t AudioFreq, void *Params);
uint8_t BSP_AUDIO_OUT_SetOutputMode(void *handle, uint8_t Output);
uint8_t BSP_AUDIO_OUT_SetMute(void *handle, uint32_t Cmd);
uint8_t BSP_AUDIO_OUT_SetVolume(void *handle, uint8_t Volume);
uint8_t BSP_AUDIO_OUT_Stop(void *handle, uint32_t Option);
/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* These functions should be implemented by the user application.*/
void    BSP_AUDIO_OUT_TransferComplete_CallBack(void);
void    BSP_AUDIO_OUT_HalfTransfer_CallBack(void);
void    BSP_AUDIO_OUT_Error_CallBack(void);
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /* __BLUECOIN_AUDIO_OUT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


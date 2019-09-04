/**
******************************************************************************
* @file    BlueCoin_audio_out.c
* @author  Central Labs
* @version V1.2.0
* @date    23-March-2018
* @brief   This file provides the Audio Out driver for the BlueCoin
*          expansion board
*******************************************************************************
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "BlueCoin_audio_out.h"
#include "BlueCoin.h"

/** @addtogroup BSP BSP
* @{
*/

/** @defgroup  BlueCoin BlueCoin
* @{
*/

/** @defgroup BlueCoin_Audio_OUT
* @brief This file provides a set of firmware functions to manage audio output codec.
* @{
*/

/** @addtogroup BlueCoin_Audio_OUT_Private_Macros  
* @{
*/

#define SAIClockDivider(__FREQUENCY__) \
  ((__FREQUENCY__ == AUDIO_FREQUENCY_8K)  ? 12 \
   : (__FREQUENCY__ == AUDIO_FREQUENCY_11K) ? 2 \
   : (__FREQUENCY__ == AUDIO_FREQUENCY_16K) ? 6 \
   : (__FREQUENCY__ == AUDIO_FREQUENCY_22K) ? 1 \
   : (__FREQUENCY__ == AUDIO_FREQUENCY_32K) ? 3 \
   : (__FREQUENCY__ == AUDIO_FREQUENCY_44K) ? 0 \
   : (__FREQUENCY__ == AUDIO_FREQUENCY_48K) ? 2 : 1)

/**
* @}
*/

/** @addtogroup BlueCoin_Audio_OUT_Private_Variables  
* @{
*/

static DrvContextTypeDef CODEC_Handle[CODEC_SENSORS_MAX_NUM ];

/**
* @}
*/

/** @addtogroup BlueCoin_Audio_OUT_Exported_Variables  
* @{
*/

SAI_HandleTypeDef                 haudio_out_sai;
/**
* @}
*/
/** @addtogroup BlueCoin_Audio_OUT_Private_Function_Prototypes  
* @{
*/
static DrvStatusTypeDef BSP_PCM1774_CODEC_Init( void **handle, uint8_t Volume, uint32_t AudioFreq);
static void SAIx_MspInit(void);
static void SAIx_Init(uint32_t AudioFreq);

/**
* @}
*/


/** @addtogroup BlueCoin_Audio_OUT_Exported_Functions
* @{
*/

/**
* @brief  Configures the audio peripherals.
* @param  OutputDevice: OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
*                       or OUTPUT_DEVICE_BOTH.
* @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
* @param  AudioFreq: Audio frequency used to play the audio stream.
* @note   The SAI PLL input clock must be done in the user application.
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_Init(CODEX_ID_t id, void **handle, uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{
  BSP_AUDIO_OUT_ClockConfig(AudioFreq, NULL);
  
  *handle = NULL;
  
  switch(id)
  {
    case CODEX_SENSORS_AUTO:
    default:
    {
      /* Try to init the LSM6DSM before */
      if(BSP_PCM1774_CODEC_Init(handle, Volume, AudioFreq) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case PCM1774_0:
    {
      if( BSP_PCM1774_CODEC_Init(handle, Volume, AudioFreq) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }
  
  /* SAI data transfer preparation:
  Prepare the Media to be used for the audio transfer from memory to SAI peripheral */
  SAIx_Init(AudioFreq);
  
  return COMPONENT_OK;
}


/**
* @brief  Starts playing audio stream from a data buffer for a determined size.
* @param  pBuffer: Pointer to the buffer
* @param  Size: Number of audio data BYTES.
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_Play(void *handle, uint16_t* pBuffer, uint32_t Size)
{
  /* Start DMA transfer of PCM samples towards the serial audio interface */
  if ( HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t *)pBuffer, DMA_MAX(Size)) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

static DrvStatusTypeDef BSP_PCM1774_CODEC_Init( void **handle, uint8_t Volume, uint32_t AudioFreq)
{
  AUDIO_DrvTypeDef *driver = NULL;
  
  if(CODEC_Handle[PCM1774_0].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }
  
  if (Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Setup handle. */
  CODEC_Handle[PCM1774_0].who_am_i      = 0;
  CODEC_Handle[PCM1774_0].ifType        = 0; // I2C interface
  CODEC_Handle[PCM1774_0].address       = PCM1774_CODEC_I2C_ADDRESS_LOW;
  CODEC_Handle[PCM1774_0].instance      = PCM1774_0;
  CODEC_Handle[PCM1774_0].isInitialized = 0;
  CODEC_Handle[PCM1774_0].isEnabled     = 0;
  CODEC_Handle[PCM1774_0].isCombo       = 1;
  CODEC_Handle[PCM1774_0].pData         = ( void * )NULL;
  CODEC_Handle[PCM1774_0].pVTable       = ( void * )&PCM1774_drv;
  CODEC_Handle[PCM1774_0].pExtVTable    = 0;
  
  *handle = (void *)&CODEC_Handle[PCM1774_0];
  
  driver = ( AUDIO_DrvTypeDef * )((DrvContextTypeDef *)(*handle))->pVTable;
  
  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }
  
  if (driver->Init( (DrvContextTypeDef *)(*handle), Volume, AudioFreq ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
* @brief  This function Pauses the audio file stream. In case
*         of using DMA, the DMA Pause feature is used.
* @note When calling BSP_AUDIO_OUT_Pause() function for pause, only
*       BSP_AUDIO_OUT_Resume() function should be called for resume
*       (use of BSP_AUDIO_OUT_Play() function for resume could lead
*       to unexpected behavior).
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_Pause(void *handle)
{
  /* Pause DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_DMAPause(&haudio_out_sai) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  
  return AUDIO_OK;
}

/**
* @brief  This function  Resumes the audio file stream.
* @note When calling BSP_AUDIO_OUT_Pause() function for pause, only
*       BSP_AUDIO_OUT_Resume() function should be called for resume
*       (use of BSP_AUDIO_OUT_Play() function for resume could lead to
*       unexpected behavior).
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_Resume(void *handle)
{
  /* Call the Audio Codec Resume function */
  /* Resume DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_DMAResume(&haudio_out_sai) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  return AUDIO_OK;
}

/**
* @brief  Stops audio playing and Power down the Audio Codec.
* @param  Option: ignored in this implementation
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_Stop(void *handle, uint32_t Option)
{
  /* Stop DMA transfer of PCM samples towards the serial audio interface */
  if (HAL_SAI_DMAStop(&haudio_out_sai) != HAL_OK)
  {
    return COMPONENT_ERROR;
  }
  return COMPONENT_OK;
}

/**
* @brief  Controls the current audio volume level.
* @param  Volume: Volume level to be set in percentage from 0 to 63 (0 for
*         Mute and 63 for Max volume level).
* @retval AUDIO_OK if correct communication, else wrong communication
*/

uint8_t BSP_AUDIO_OUT_SetVolume(void *handle, uint8_t Volume)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  AUDIO_DrvTypeDef *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( AUDIO_DrvTypeDef *)ctx->pVTable;
  
  if(Volume == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->SetVolume == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->SetVolume(ctx, Volume ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
* @brief  Enables or disables the MUTE mode by software
* @param  Cmd: Could be AUDIO_MUTE_ON to mute sound or AUDIO_MUTE_OFF to
*         unmute the codec and restore previous volume level.
* @retval AUDIO_OK if correct communication, else wrong communication
*/
uint8_t BSP_AUDIO_OUT_SetMute(void *handle, uint32_t Cmd)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  AUDIO_DrvTypeDef *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( AUDIO_DrvTypeDef *)ctx->pVTable;

  if ( driver->SetMute == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->SetMute(ctx, Cmd ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
* @brief  Clock Config.
* @param  Params: additional parameters where required
* @param  AudioFreq: Audio frequency used to play the audio stream.
* @note   This API is called by BSP_AUDIO_OUT_Init()
*         Being __weak it can be overwritten by the application
* @retval AUDIO_OK if no problem during execution, AUDIO_ERROR otherwise
*/
__weak uint8_t BSP_AUDIO_OUT_ClockConfig(uint32_t AudioFreq, void *Params)
{
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  
  /* Configure PLLSAI prescalers */
  /* PLLSAI_VCO: VCO_429M
     SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
     SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 214.5/19 = 11.289 Mhz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  RCC_PeriphCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIM = 16;
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIN = 344;
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIQ = 7;
  RCC_PeriphCLKInitStruct.PLLSAIDivQ = 1;
  
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
  
  return AUDIO_OK;
  
  
}

/**
* @brief  Update the audio frequency.
* @param  AudioFreq: Audio frequency used to play the audio stream.
* @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
*         audio frequency.
* @retval None
*/
uint8_t BSP_AUDIO_OUT_SetFrequency(void *handle, uint32_t AudioFreq)
{
  BSP_AUDIO_OUT_ClockConfig(AudioFreq, NULL);
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);
  
  /* Update the SAI audio frequency configuration */
  haudio_out_sai.Init.Mckdiv = SAIClockDivider(AudioFreq);
  HAL_SAI_Init(&haudio_out_sai);
  
  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
  
  return AUDIO_OK;
}


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm324x9i_eval_audio.h) */
  BSP_AUDIO_OUT_TransferComplete_CallBack();
}
/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  hsai: SAI handle
  * @retval None
  */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm324x9i_eval_audio.h) */
  BSP_AUDIO_OUT_HalfTransfer_CallBack();
}

/**
* @brief  SAI error callbacks.
* @param  hsai: SAI handle
* @retval None
*/
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  BSP_AUDIO_OUT_Error_CallBack();
}

/**
* @brief  Manages the DMA full Transfer complete event.
* @retval None
*/
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
}

/**
* @brief  Manages the DMA Half Transfer complete event.
* @retval None
*/
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
}

/**
* @brief  Manages the DMA FIFO error event.
* @retval None
*/
__weak void BSP_AUDIO_OUT_Error_CallBack(void)
{
}

/**
* @}
*/


/**
* @brief  Initializes SAI MSP.
* @retval None
*/
static void SAIx_MspInit(void)
{
  static DMA_HandleTypeDef hdma_saiTx;
  GPIO_InitTypeDef  GPIO_InitStruct;
  SAI_HandleTypeDef *hsai = &haudio_out_sai;
  
  /* Enable SAI clock */
  AUDIO_SAIx_CLK_ENABLE();
  
  /* Enable GPIO clock */
  AUDIO_SAIx_SCK_ENABLE();
  AUDIO_SAIx_SD_ENABLE();
  AUDIO_SAIx_MCLK_ENABLE();
  AUDIO_SAIx_FS_ENABLE();

  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_SAIx_MCLK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = AUDIO_SAIx_MCLK_SCK_SD_FS_AF;
  HAL_GPIO_Init(AUDIO_SAIx_MCLK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AUDIO_SAIx_SCK_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AUDIO_SAIx_FS_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_FS_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AUDIO_SAIx_SD_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_SD_GPIO_PORT, &GPIO_InitStruct);
  
  /* Enable the DMA clock */
  AUDIO_SAIx_DMAx_CLK_ENABLE();
  
  if(hsai->Instance == AUDIO_SAIx)
  {
    /* Configure the hdma_saiTx handle parameters */
    hdma_saiTx.Init.Channel             = AUDIO_SAIx_DMAx_CHANNEL;
    hdma_saiTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_saiTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_saiTx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_saiTx.Init.PeriphDataAlignment = AUDIO_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_saiTx.Init.MemDataAlignment    = AUDIO_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_saiTx.Init.Mode                = DMA_CIRCULAR;
    hdma_saiTx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_saiTx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_saiTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_saiTx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_saiTx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    
    hdma_saiTx.Instance = AUDIO_SAIx_DMAx_STREAM;
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_saiTx);
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_saiTx);
    
    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_saiTx);
  }
  
  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_SAIx_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_SAIx_DMAx_IRQ);
}

/**
* @brief  Initializes the Audio Codec audio interface (SAI).
* @param  AudioFreq: Audio frequency to be configured for the SAI peripheral.
* @note   The default SlotActive configuration is set to CODEC_AUDIOFRAME_SLOT_0123
*         and user can update this configuration using
* @retval None
*/
static void SAIx_Init(uint32_t AudioFreq)
{
  /* Initialize the haudio_out_sai Instance parameter */
  haudio_out_sai.Instance = AUDIO_SAIx;
  
  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);
  
  /* Configure SAI_Block_x
  LSBFirst: Disabled
  DataSize: 16 */
  haudio_out_sai.Init.AudioFrequency = AudioFreq;
  haudio_out_sai.Init.ClockSource = SAI_CLKSOURCE_PLLSAI;//SAI_CLKSOURCE_PLLI2S;
  haudio_out_sai.Init.AudioMode = SAI_MODEMASTER_TX;
  haudio_out_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLED;
  haudio_out_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_out_sai.Init.DataSize = SAI_DATASIZE_16;
  haudio_out_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_out_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  haudio_out_sai.Init.Synchro = SAI_ASYNCHRONOUS;
  haudio_out_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLED;
  haudio_out_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  
  /* Configure SAI_Block_x Frame
  Frame Length: 64
  Frame active Length: 32
  FS Definition: Start frame + Channel Side identification
  FS Polarity: FS active Low
  FS Offset: FS asserted one bit before the first bit of slot 0 */
  haudio_out_sai.FrameInit.FrameLength = 32;
  haudio_out_sai.FrameInit.ActiveFrameLength = 16;
  haudio_out_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_out_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_out_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  
  /* Configure SAI Block_x Slot
  Slot First Bit Offset: 0
  Slot Size  : 16
  Slot Number: 4
  Slot Active: All slot actives */
  haudio_out_sai.SlotInit.FirstBitOffset = 0;
  haudio_out_sai.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
  haudio_out_sai.SlotInit.SlotNumber = 2;
  haudio_out_sai.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;
  if(HAL_SAI_GetState(&haudio_out_sai) == HAL_SAI_STATE_RESET)
  {
    /* Init the SAI */
    SAIx_MspInit();
  }
  HAL_SAI_Init(&haudio_out_sai);
  
  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
}
/**************************/

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

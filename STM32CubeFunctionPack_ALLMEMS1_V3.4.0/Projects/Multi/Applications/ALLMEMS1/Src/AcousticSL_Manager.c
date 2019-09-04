/**
 ******************************************************************************
 * @file    AcousticSL_Manager.c
 * @author  Central LAB
 * @version V3.4.0
 * @date    26-Apr-2018
 * @brief   This file includes Source location interface functions
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

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION

/* Volatile variables and its initialization --------------------------------------*/
volatile int32_t SourceLocationToSend=  ACOUSTIC_SL_NO_AUDIO_DETECTED;

/* Exported variables -------------------------------------------------------------*/
/*Handler and Config structure for Source Localization*/
AcousticSL_Handler_t libSoundSourceLoc_Handler_Instance;
AcousticSL_Config_t  libSoundSourceLoc_Config_Instance;

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PCM_Buffer[];

/* Private function prototypes -----------------------------------------------*/
static void SW_Task2_Start(void);

/**
* @brief  Initialises AcousticSL algorithm
* @param  None
* @retval None
*/
void AcousticSL_Manager_init(void)
{
  char LibVersion[36];
  
  AcousticSL_GetLibVersion(LibVersion);
  
  /*Setup Source Localization static parameters*/
  libSoundSourceLoc_Handler_Instance.channel_number= AUDIO_CHANNELS;

#ifdef STM32_NUCLEO  
  libSoundSourceLoc_Handler_Instance.M12_distance= SIDE;
#endif /* STM32_NUCLEO */
  
#ifdef STM32_BLUECOIN
  libSoundSourceLoc_Handler_Instance.M12_distance =DIAGONAL;
  libSoundSourceLoc_Handler_Instance.M34_distance =DIAGONAL;
#endif /* STM32_BLUECOIN */
  
  libSoundSourceLoc_Handler_Instance.sampling_frequency= AUDIO_SAMPLING_FREQUENCY;  
  libSoundSourceLoc_Handler_Instance.algorithm= ACOUSTIC_SL_ALGORITHM_GCCP;
  libSoundSourceLoc_Handler_Instance.ptr_M1_channels= AUDIO_CHANNELS;
  libSoundSourceLoc_Handler_Instance.ptr_M2_channels= AUDIO_CHANNELS;
  libSoundSourceLoc_Handler_Instance.ptr_M3_channels= AUDIO_CHANNELS; 
  libSoundSourceLoc_Handler_Instance.ptr_M4_channels= AUDIO_CHANNELS;
  libSoundSourceLoc_Handler_Instance.samples_to_process = 512;

  AcousticSL_getMemorySize( &libSoundSourceLoc_Handler_Instance);  
  libSoundSourceLoc_Handler_Instance.pInternalMemory=(uint32_t *)malloc(libSoundSourceLoc_Handler_Instance.internal_memory_size);
  
  if(libSoundSourceLoc_Handler_Instance.pInternalMemory == NULL)
  {
    ALLMEMS1_PRINTF("Error AcousticSL Memory allocation\n\r");
    return;
  }
  
  if(AcousticSL_Init( &libSoundSourceLoc_Handler_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticSL Initialization\n\r");
    return;
  }

  /*Setup Source Localization dynamic parameters*/  
  libSoundSourceLoc_Config_Instance.resolution= ANGLE_RESOLUTION;
  libSoundSourceLoc_Config_Instance.threshold=  SENSITIVITY_SL_HI_THRESHOLD;

  if(AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance, &libSoundSourceLoc_Config_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticSL Configuration\n\r");
    return;
  }
     
  /* If everything is ok */
  TargetBoardFeatures.AcousticSLIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s (%ld bytes allocated)\r\n", LibVersion, libSoundSourceLoc_Handler_Instance.internal_memory_size);
 
  HAL_NVIC_SetPriority((IRQn_Type)EXTI2_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI2_IRQn);

  return;
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
*         In this application only PDM to PCM conversion and USB streaming
*         is performed.
*         User can add his own code here to perform some DSP or audio analysis.
* @param  none
* @retval None
*/
void AudioProcess_SL(void)
{
#ifdef STM32_NUCLEO
  if(AcousticSL_Data_Input((int16_t *)&PCM_Buffer[M1_EXT_B],
                           (int16_t *)&PCM_Buffer[M4_EXT_B],
                           NULL,
                           NULL,
                           &libSoundSourceLoc_Handler_Instance))
#endif /* STM32_NUCLEO */
    
#ifdef STM32_BLUECOIN     
  if(AcousticSL_Data_Input((int16_t *)&PCM_Buffer[BOTTOM_LEFT_MIC],
                           (int16_t *)&PCM_Buffer[TOP_RIGHT_MIC],
                           (int16_t *)&PCM_Buffer[BOTTOM_RIGHT_MIC],
                           (int16_t *)&PCM_Buffer[TOP_LEFT_MIC],
                           &libSoundSourceLoc_Handler_Instance))
#endif /* STM32_BLUECOIN */
  {
    /*Localization Processing Task*/
    SW_Task2_Start(); 
  }
}

void SetConfig_SL(uint16_t newThresh)
{
  libSoundSourceLoc_Config_Instance.threshold = newThresh;
  AcousticSL_setConfig(&libSoundSourceLoc_Handler_Instance,&libSoundSourceLoc_Config_Instance);
}

/**
* @brief Throws Highest priority interrupt
* @param  None
* @retval None
*/
void SW_Task2_Start(void)
{
  HAL_NVIC_SetPendingIRQ(EXTI2_IRQn);
}

/**
* @brief Lower priority interrupt handler routine
* @param  None
* @retval None
*/
void  SW_Task2_Callback(void) 
{
  int32_t result=0;
  
  if(AcousticSL_Process((int32_t *)&result, &libSoundSourceLoc_Handler_Instance))
  {
    ALLMEMS1_PRINTF("AcousticSL_Process error\r\n");
    return;
  }
  
  if(result != ACOUSTIC_SL_NO_AUDIO_DETECTED)
  {
    SourceLocationToSend= (int16_t)result;
    
#ifdef STM32_NUCLEO    
    SourceLocationToSend= SourceLocationToSend + 90;
#endif /* STM32_NUCLEO */
    
#ifdef STM32_BLUECOIN      
    SourceLocationToSend= SourceLocationToSend - 45;
    if(SourceLocationToSend<0)
      SourceLocationToSend=SourceLocationToSend + 360;
#endif /* STM32_BLUECOIN */
  }
}

#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

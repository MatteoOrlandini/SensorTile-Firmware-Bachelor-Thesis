/**
 ******************************************************************************
 * @file    AcousticBF_Manager.c
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
#include "sensor_service.h"
#include "AcousticBF_Manager.h"

#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING

/* Imported Variables -------------------------------------------------------------*/
#ifdef USE_STM32F4XX_NUCLEO
extern uint16_t PDM_Buffer[];
#endif /* USE_STM32F4XX_NUCLEO */

/* Private variables -------------------------------------------------------------*/
/*Handler and Config structure for BeamForming*/
AcousticBF_Handler_t libBeamforming_Handler_Instance;
AcousticBF_Config_t lib_Beamforming_Config_Instance;

static uint8_t MicInputBF_1;
static uint8_t MicInputBF_2;

int16_t BFOUT[AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY/1000];

/* Private function prototypes ---------------------------------------------------*/
void SW_Task1_Start(void);

/**
* @brief  Initialize Acoustic Beam Forming library
* @param  None
* @retval None
*/
void AcousticBF_Manager_init(void)
{
  char LibVersion[36];
  
  AcousticBF_GetLibVersion(LibVersion);
  
  /*Setup Beamforming static parameters*/
  
#ifdef STM32_NUCLEO
  #ifdef STM32F446xx
      libBeamforming_Handler_Instance.algorithm_type_init= ACOUSTIC_BF_TYPE_STRONG;
  #elif STM32F401xE
      libBeamforming_Handler_Instance.algorithm_type_init= ACOUSTIC_BF_TYPE_ASR_READY;
  #endif /* STM32F446xx */
#elif STM32_BLUECOIN
  libBeamforming_Handler_Instance.algorithm_type_init= ACOUSTIC_BF_TYPE_STRONG;
#endif /* STM32_NUCLEO */
  
  libBeamforming_Handler_Instance.ref_mic_enable= ACOUSTIC_BF_REF_ENABLE;
  libBeamforming_Handler_Instance.ptr_out_channels= OUTPUT_STREAM_NUMBER_CHANNELS;
  libBeamforming_Handler_Instance.data_format= ACOUSTIC_BF_DATA_FORMAT_PDM;
  libBeamforming_Handler_Instance.sampling_frequency= AUDIO_SAMPLING_FREQUENCY_BF;  
  libBeamforming_Handler_Instance.ptr_M1_channels= AUDIO_CHANNELS;
  libBeamforming_Handler_Instance.ptr_M2_channels= AUDIO_CHANNELS;
  libBeamforming_Handler_Instance.delay_enable = 1;
  
  AcousticBF_getMemorySize(&libBeamforming_Handler_Instance);
  libBeamforming_Handler_Instance.pInternalMemory =(uint32_t *)malloc(libBeamforming_Handler_Instance.internal_memory_size);
  
  if(libBeamforming_Handler_Instance.pInternalMemory == NULL)
  {
    ALLMEMS1_PRINTF("Error AcousticBF Memory allocation\n\r");
    return;
  }
  
  if(AcousticBF_Init(&libBeamforming_Handler_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticBF Initialization\n\r");
    return;
  }  
  
  /*Setup Beamforming dynamic parameters*/
  
#ifdef STM32_NUCLEO
  #ifdef STM32F446xx
      lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_STRONG;
  #elif STM32F401xE
      lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_ASR_READY;
  #endif /* STM32F446xx */
  lib_Beamforming_Config_Instance.mic_distance= SIDE;
#elif STM32_BLUECOIN
  lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_ASR_READY;
  lib_Beamforming_Config_Instance.mic_distance= DIAGONAL;
#endif /* STM32_NUCLEO */
  
  lib_Beamforming_Config_Instance.M2_gain= GAIN_APPLIED_TO_SECOND_MICROPHONE;  
  lib_Beamforming_Config_Instance.volume = AUDIO_BF_VOLUME_VALUE;

  if(AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance))
  {
    ALLMEMS1_PRINTF("Error AcousticBF Configuration\n\r");
    return;
  }

/* Set direction default value (case 3) */ 
#ifdef STM32_NUCLEO
  MicInputBF_1= M1_EXT_B;
  MicInputBF_2= M4_EXT_B;
#elif STM32_BLUECOIN
  MicInputBF_1= BOTTOM_RIGHT_MIC;
  MicInputBF_2= BOTTOM_LEFT_MIC;
#endif /* STM32_NUCLEO */
    
  /* If everything is ok */
  TargetBoardFeatures.AcousticBFIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s (%ld bytes allocated)\r\n", LibVersion, libBeamforming_Handler_Instance.internal_memory_size);
  
  HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 0x08, 0);  
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn); 	
}

/**
* @brief  Support function needed to setup the correct beamforming microphones depending
*         on the direction choosen by the user.
* @param  Direction: parameter that describes the desired output:
*         For nucleo Board: 1 to 2 for one of the possible beamforming directions* 
*         For BlueCoin:     1 to 8 for one of the possible beamforming directions*                         
* @retval None
*/
void BeamFormingDirectionSetup(int32_t Direction)
{
#ifdef STM32_NUCLEO
  switch(Direction)
  {
    case 7://LEFT
      MicInputBF_1= M4_EXT_B;
      MicInputBF_2= M1_EXT_B;
      break;
    case 3://RIGHT
      MicInputBF_1= M1_EXT_B;
      MicInputBF_2= M4_EXT_B;
      break;
  }
#elif STM32_BLUECOIN
 switch(Direction)
  {
    case 1:
      MicInputBF_1= TOP_RIGHT_MIC;
      MicInputBF_2= BOTTOM_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE;
    break;
    case 2:
      MicInputBF_1= TOP_RIGHT_MIC;
      MicInputBF_2= BOTTOM_LEFT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
    break;
    case 3:
      MicInputBF_1= BOTTOM_RIGHT_MIC;
      MicInputBF_2= BOTTOM_LEFT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE;
    break;
    case 4:
      MicInputBF_1= BOTTOM_RIGHT_MIC;
      MicInputBF_2= TOP_LEFT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
    break;
    case 5:
      MicInputBF_1= BOTTOM_RIGHT_MIC;
      MicInputBF_2= TOP_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE;
    break;
    case 6:
      MicInputBF_1= BOTTOM_LEFT_MIC;
      MicInputBF_2= TOP_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
    break;
    case 7:
      MicInputBF_1= BOTTOM_LEFT_MIC;
      MicInputBF_2= BOTTOM_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = SIDE;
    break;
    case 8:
      MicInputBF_1= TOP_LEFT_MIC;
      MicInputBF_2= BOTTOM_RIGHT_MIC;
      lib_Beamforming_Config_Instance.mic_distance = DIAGONAL;
    break;
  }
  
  AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance);
#endif /* STM32_NUCLEO */
}

void BeamFormingSetType(uint32_t type)
{  
//    BV_ADPCM_Profile_Status status;
//
//  status =BluevoiceADPCM_GetStatus();
  //while(BluevoiceADPCM_GetStatus() != BV_ADPCM_STATUS_READY);

  if(type == 0)
  {
    lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_ASR_READY;
    if(AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance))
    {
      ALLMEMS1_PRINTF("Error AcousticBF Configuration\n\r");
      return;
    } 
    
  }
  else if(type == 1)
  {
    lib_Beamforming_Config_Instance.algorithm_type= ACOUSTIC_BF_TYPE_STRONG;
    if(AcousticBF_setConfig(&libBeamforming_Handler_Instance, &lib_Beamforming_Config_Instance))
    {
      ALLMEMS1_PRINTF("Error AcousticBF Configuration\n\r");
      return;
    } 
    
  }
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess_BF(int32_t dir)
{
  if(AcousticBF_FirstStep(&((uint8_t *)(PDM_Buffer))[MicInputBF_1],
                          &((uint8_t *)(PDM_Buffer))[MicInputBF_2], 
                          (int16_t *)BFOUT,
                          &libBeamforming_Handler_Instance))
  {
    SW_Task1_Start();          
  }
}

/**
* @brief Throws Highest priority interrupt
* @param  None
* @retval None
*/
void SW_Task1_Start(void)
{  
  HAL_NVIC_SetPendingIRQ(EXTI1_IRQn);  
}

/**
* @brief  Highest priority interrupt handler routine
* @param  None
* @retval None
*/
void SW_Task1_Callback(void)
{        
  AcousticBF_SecondStep(&libBeamforming_Handler_Instance); 
}

#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
 ******************************************************************************
 * @file    AudioBV_Manager.c
 * @author  Central LAB
 * @version V3.4.0
 * @date    26-Apr-2018
 * @brief   This file includes BlueVoice interface functions
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
//#include "stm32F4xx_hal.h"

/* Code for BlueVoice integration - Start Section */

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PCM_Buffer[];
extern BV_ADPCM_ProfileHandle_t BLUEVOICE_tx_handle;

#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
extern volatile uint8_t BF_toggleFlag;
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
extern uint8_t IsSdMemsRecording;
extern uint8_t IsSdAudioRecording;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/* Private Variables -------------------------------------------------------------*/
static uint32_t led_toggle_count = 0; /*!< Variable used to handle led toggling.*/
BV_ADPCM_Config_t BLUEVOICE_Config;
BV_ADPCM_Status bvStat;

extern volatile uint16_t BFOUT[];
volatile int32_t BF_dir;

/* Code for BlueVoice integration - Start Section */
static uint16_t num_byte_sent = 0;
/* Code for BlueVoice integration - End Section */

//#if defined(STM32F401xE) || defined(STM32_BLUECOIN)  
void SW_Task3_Start(void);
//#endif

/* Private Defines -------------------------------------------------------------*/
#define LED_TOGGLE_STREAMING  100


/**
* @brief  Initialises BlueVoice manager
* @param  None
* @retval None
*/
void AudioBV_Manager_init(void)
{
  BluevoiceADPCM_Initialize();
  
#ifdef STM32_NUCLEO
  BLUEVOICE_Config.sampling_frequency = FR_8000;
  BLUEVOICE_Config.channel_in = 1;
  BLUEVOICE_Config.channel_tot = 2;
  bvStat = BluevoiceADPCM_SetConfig(&BLUEVOICE_Config);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
  HAL_NVIC_SetPriority((IRQn_Type)EXTI3_IRQn, 0x07, 0); 
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI3_IRQn); 
  
#endif /* STM32_NUCLEO */
  
#ifdef STM32_SENSORTILE
  BLUEVOICE_Config.sampling_frequency = FR_8000;
  BLUEVOICE_Config.channel_in = 1;
  BLUEVOICE_Config.channel_tot = 1;
  bvStat = BluevoiceADPCM_SetConfig(&BLUEVOICE_Config);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
  HAL_NVIC_SetPriority((IRQn_Type)EXTI3_IRQn, 0x07, 0); 
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI3_IRQn); 
#endif /* STM32_SENSORTILE */

#ifdef STM32_BLUECOIN
  BLUEVOICE_Config.sampling_frequency = FR_8000;
  BLUEVOICE_Config.channel_in = 1;
  BLUEVOICE_Config.channel_tot = 4;
  bvStat = BluevoiceADPCM_SetConfig(&BLUEVOICE_Config);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
  HAL_NVIC_SetPriority((IRQn_Type)EXTI3_IRQn, 0x07, 0); 
  HAL_NVIC_EnableIRQ((IRQn_Type)EXTI3_IRQn); 
#endif /* STM32_BLUECOIN */
  
  bvStat = BluevoiceADPCM_SetTxHandle(&BLUEVOICE_tx_handle);
  if (bvStat != BV_ADPCM_SUCCESS) {
    goto fail;
  }
  
  /* If everything is ok */
  TargetBoardFeatures.AudioBVIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized ST BlueVoiceADPCM v2.0.0\r\n");

  return;
  
  fail:
    while(1){}
}

/**
* @brief  User function that is called when the PCM_Buffer is full and ready to send.
* @param  none
* @retval None
*/
void AudioProcess_BV(void)
{
  BV_ADPCM_Status status;
  
#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
  if(BF_toggleFlag==TRUE){
    AudioProcess_BF(BF_dir);
  }
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */

 
  if (BluevoiceADPCM_IsProfileConfigured())
  {
#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
    if(BF_toggleFlag==TRUE)
    {
      status = BluevoiceADPCM_AudioIn( (uint16_t*)/*BF_Buffer*/BFOUT, BV_PCM_AUDIO_IN_SAMPLES);
    }
    else
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */
    {
      status = BluevoiceADPCM_AudioIn( (uint16_t*)PCM_Buffer, BV_PCM_AUDIO_IN_SAMPLES); 
    }

#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING    
    if(BF_toggleFlag==FALSE)
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */
    {
      if (led_toggle_count++ >= LED_TOGGLE_STREAMING) 
      {
        led_toggle_count = 0;
		
        #ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
        if( (!IsSdMemsRecording) && (!IsSdAudioRecording))
        {
        #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
          LedToggleTargetPlatform();
        #ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
        }
        #endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */
      }
    }
    
    if(status==BV_ADPCM_OUT_BUF_READY) {
      SW_Task3_Start();
    }
  }
  
}

#if (defined(STM32F401xE) || defined(STM32F446xx))
/**
* @brief 
* @param None
* @retval None
*/
void BV_SetInForBF(void)
{
  BLUEVOICE_Config.channel_tot = 4;

  while(BluevoiceADPCM_SetConfig(&BLUEVOICE_Config) != BV_ADPCM_SUCCESS);
}

/**
* @brief 
* @param None
* @retval None
*/
void BV_SetOutForBF(void)
{
  BLUEVOICE_Config.channel_tot = 2;
    
  while(BluevoiceADPCM_SetConfig(&BLUEVOICE_Config) != BV_ADPCM_SUCCESS);
}
#endif /* (defined(STM32F401xE) || defined(STM32F446xx)) */



/**
* @brief Throws Highest priority interrupt
* @param None
* @retval None
*/
void SW_Task3_Start(void)
{ 
  HAL_NVIC_SetPendingIRQ(EXTI3_IRQn); 
}

void SW_BV_send_Callback(void)
{
            
  BluevoiceADPCM_SendData(&num_byte_sent);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

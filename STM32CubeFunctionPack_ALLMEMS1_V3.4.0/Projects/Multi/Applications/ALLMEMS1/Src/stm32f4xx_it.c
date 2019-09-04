/**
  ******************************************************************************
  * @file    stm32f4xx_it.c 
  * @author  Central LAB
  * @version V3.4.0
  * @date    26-Apr-2018
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f4xx_it.h"

/* Imported variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef    TimEnvHandle;
extern TIM_HandleTypeDef    TimCCHandle;
extern TIM_HandleTypeDef    TimAudioDataHandle;
extern I2S_HandleTypeDef    hAudioInI2s;
    
#ifdef STM32_BLUECOIN
extern TIM_HandleTypeDef    TimBattPlugHandle;
#endif /* STM32_BLUECOIN */

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
extern TIM_HandleTypeDef    TimSdRecordingHandle;
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

#ifdef STM32_BLUECOIN
extern I2C_HandleTypeDef I2C_ONBOARD_SENSORS_Handle;
#endif /* STM32_BLUECOIN */

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  
#ifdef STM32_BLUECOIN
  BSP_Watchdog_Refresh();
#endif /* STM32_BLUECOIN */
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM4 interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimEnvHandle);
}

/**
  * @brief  This function handles TIM5 interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{  
  HAL_TIM_IRQHandler(&TimAudioDataHandle);
}

/**
  * @brief  This function handles TIM1 Interrupt request
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimCCHandle);
}

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING
/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimSdRecordingHandle);
}
#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void AUDIO_IN_I2S_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}

#ifdef ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION
/**
* @brief  This function handles External line 2 interrupt request.
* @param  None
* @retval None
*/
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  SW_Task2_Callback();
}
#endif /* ALLMEMS1_ACOUSTIC_SOURCE_LOCALIZATION */

#ifdef ALLMEMS1_ACOUSTIC_BEAM_FORMING
/**
* @brief  This function handles EXTI1 interrupt. 
* @param  None
* @retval None
*/
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  SW_Task1_Callback();
}
#endif /* ALLMEMS1_ACOUSTIC_BEAM_FORMING */

/**
* @brief  This function handles External line 2 interrupt request.
* @param  None
* @retval None
*/
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  SW_BV_send_Callback();

}

#ifdef STM32_NUCLEO

/**
  * @brief  EXTI0_IRQHandler This function handles External line
  *         interrupt request for BlueNRG.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(SPI1_CMN_DEFAULT_IRQ_PIN);
}

/**
* @brief  This function handles External line 10-15 interrupt request.
* @param  None
* @retval None
*/
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
}

#ifdef IKS01A1
/**
  * @brief  This function handles External line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler( void )
{
  HAL_GPIO_EXTI_IRQHandler(M_INT1_PIN);  
}
#elif IKS01A2
/**
  * @brief  This function handles External line 5-9 interrupt request
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler( void )
{
  HAL_GPIO_EXTI_IRQHandler(LSM6DSL_INT1_O_PIN);
}
#endif /* IKS01A1 */

#elif STM32_BLUECOIN

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimBattPlugHandle);
}

/**
  * @brief  This function handles I2C_3 interrupt request.
  * @param  None
  * @retval None
  */
void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&I2C_ONBOARD_SENSORS_Handle);
}

/**
  * @brief  This function handles I2C_3 interrupt request.
  * @param  None
  * @retval None
  */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&I2C_ONBOARD_SENSORS_Handle);
}

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  if(BSP_PB_GetState(BUTTON_1))
  {
    HAL_GPIO_EXTI_IRQHandler(BUTTON_1_PIN);
  }
  else if(BSP_PB_GetState(BUTTON_2))
  {
    HAL_GPIO_EXTI_IRQHandler(BUTTON_2_PIN);
  }  
}

/**
  * @brief  EXTI0_IRQHandler This function handles External line
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
  * @brief  This function handles External line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
  * @brief  BNRG_SPI_EXTI_IRQHandler This function handles External line
  *         interrupt request for BlueNRG.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  
  if(__HAL_GPIO_EXTI_GET_FLAG(CHRG_PIN))
  {
    HAL_GPIO_EXTI_IRQHandler(CHRG_PIN);
  }
  
  if(__HAL_GPIO_EXTI_GET_FLAG(BNRG_SPI_EXTI_PIN))
  {
    HAL_GPIO_EXTI_IRQHandler(BNRG_SPI_EXTI_PIN);
  }
}

/**
  * @brief  This function handles the PVD Output interrupt request.
  * @param  None
  * @retval None
  */
void PVD_IRQHandler(void)
{
  HAL_PWR_PVD_IRQHandler();
}

#endif /* STM32_NUCLEO */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

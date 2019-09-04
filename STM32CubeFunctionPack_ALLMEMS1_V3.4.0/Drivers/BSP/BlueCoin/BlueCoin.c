/**
******************************************************************************
* @file    BlueCoin.c
* @author  Central Labs
* @version V1.2.0
* @date    23-March-2018
* @brief   This file provides low level functionalities for BlueCoin board.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* File Info: ------------------------------------------------------------------


------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "BlueCoin.h"

/** @addtogroup BSP
* @{
*/

/** @defgroup BlueCoin
* @{
*/

/** @defgroup BlueCoin_COMMON BlueCoin_COMMON
* @{
*/


/** @defgroup BlueCoin_COMMON_Private_Defines BlueCoin_COMMON_Private_Defines
* @{
*/


/**
* @brief BlueCoin BSP Driver version number V1.0.0
*/
#define __BlueCoin_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __BlueCoin_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __BlueCoin_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __BlueCoin_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __BlueCoin_BSP_VERSION         ((__BlueCoin_BSP_VERSION_MAIN << 24)\
                                        |(__BlueCoin_BSP_VERSION_SUB1 << 16)\
                                        |(__BlueCoin_BSP_VERSION_SUB2 << 8 )\
                                        |(__BlueCoin_BSP_VERSION_RC))



/**
* @}
*/


I2C_HandleTypeDef    I2C_ONBOARD_SENSORS_Handle;
ADC_HandleTypeDef    AdcHandle;
IWDG_HandleTypeDef   IwdgHandle;


/** @defgroup BlueCoin_COMMON_Private_Variables BlueCoin_COMMON_Private_Variables
* @{
*/
static uint32_t msLastTicks[2] = {1,1};
volatile static ChrgStatus_t ChrgStatus;
volatile uint8_t IT_I2C_error_af=0;					
__IO uint16_t uhADCxConvertedValue = 0;
volatile static uint8_t BatMSEnable=0;

GPIO_TypeDef* GPIO_PORT[LEDn] =
{
  LED1_GPIO_PORT,
  LED2_GPIO_PORT,
  LED3_GPIO_PORT,
  LED4_GPIO_PORT,
  LED5_GPIO_PORT,
  LED6_GPIO_PORT,
  LED7_GPIO_PORT,
  LED8_GPIO_PORT
};

const uint32_t GPIO_PIN[LEDn] =
{
  LED1_PIN,
  LED2_PIN,
  LED3_PIN,
  LED4_PIN,
  LED5_PIN,
  LED6_PIN,
  LED7_PIN,
  LED8_PIN
};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] =
{
  BUTTON_1_GPIO_PORT,
  BUTTON_2_GPIO_PORT
};

const uint16_t BUTTON_PIN[BUTTONn] =
{
  BUTTON_1_PIN,
  BUTTON_2_PIN
};

const uint16_t BUTTON_IRQn[BUTTONn] =
{
  BUTTON_1_EXTI_IRQn,
  BUTTON_2_EXTI_IRQn
};

/**
* @}
*/


static HAL_StatusTypeDef        I2C_ONBOARD_SENSORS_Init(void);
static HAL_StatusTypeDef        I2C_ONBOARD_SENSORS_WriteData(uint8_t* pBuffer, uint8_t  DevAddr, uint16_t RegAdd,uint8_t RegAddSize, uint16_t Size);
static HAL_StatusTypeDef        I2C_ONBOARD_SENSORS_ReadData(uint8_t* pBuffer, uint8_t  DevAddr, uint16_t RegAdd, uint8_t RegAddSize, uint16_t Size);
static void                     I2C_ONBOARD_SENSORS_MspInit(void);
static void                     I2C_ONBOARD_SENSORS_Error(void);
static void                     ADC_BLUECOIN_MspInit(void);
static void                     ADC_BLUECOIN_MspDeInit(void);
void                            LSM6DSM_Sensor_IO_ITConfig( void );



/** @defgroup BlueCoin_COMMON_Private_Functions BlueCoin_COMMON_Public_Functions
* @{
*/

/**
* @brief  This method returns the BlueCoin EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __BlueCoin_BSP_VERSION;
}

/**
* @brief  This function confgures the GPIO which manages the shutdown pin
* @param  None
* @retval version:
*/
void BSP_ShutDown_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED clock */
  SHUTDOWN_GPIO_CLK_ENABLE();
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = SHUTDOWN_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  /* Reset the pin before init to prevent glitches */
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
* @brief  This function manages the shutdown pin of STBC03J turning off the board
* @param  None
* @retval version
*/
void BSP_ShutDown(void)
{
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_GPIO_PIN, GPIO_PIN_SET);
}


/**
* @brief Confgure the STM32 internal watchdog. IWDG counter clock Period
* is set to 4ms, watchdog timeout is 4ms*(ReloadN+1).
* @param  ReloadN: IWDG down-counter reload value [0-4095]
* @retval version:
*/
DrvStatusTypeDef BSP_Watchdog_Init(uint16_t ReloadN)
{
  /* Set counter reload value to obtain 4ms*(ReloadN+1) IWDG TimeOut.
     IWDG counter clock Period = 1 / (LsiFreq / Prescaler) = 1/(32KHz/128) = 4ms */
  IwdgHandle.Instance = IWDG;

  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_128;
  IwdgHandle.Init.Reload    = (ReloadN&0xFFF);

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
* @brief  Refresh the STM32 internal watchdog
* @param  none
* @retval none
*/
void BSP_Watchdog_Refresh()
{
  HAL_IWDG_Refresh(&IwdgHandle);
}


/**
* @brief  This method init the GPIO for charger status routine
* @param  None
* @retval none
*/
void BSP_ChrgPin_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED clock */
  CHRG_GPIO_CLK_ENABLE();
 
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = CHRG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(CHRG_GPIO_PORT, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(CHRG_EXTI_IRQn, 3, 0x00);
  HAL_NVIC_EnableIRQ(CHRG_EXTI_IRQn);
}

/**
* @brief  This method get the charger status of the board
* @param  None
* @retval Charger status
*/
ChrgStatus_t BSP_GetChrgStatus(void)
{  
  uint32_t timeElapsed = HAL_GetTick()-msLastTicks[0];
 
  if(timeElapsed > 1000 && HAL_GPIO_ReadPin(CHRG_GPIO_PORT,CHRG_PIN) == GPIO_PIN_SET)
  {
    ChrgStatus = CHRG_STATUS_DISCHARGING;
  }
  return ChrgStatus;
}

/**
* @brief Update board charging status value
* @param  system tick
* @retval None
*/
void BSP_SetLastChrgTick(uint32_t msTick)
{
  float freqs[2];
  
  freqs[0] = 1000/(((float)msTick-(float)msLastTicks[0]));
  freqs[1] = 1000/((float)msLastTicks[0]-(float)msLastTicks[1]);
  
  msLastTicks[1]=msLastTicks[0];
  msLastTicks[0]=msTick;
    
  if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_EOC,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_EOC,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_EOC;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_PRE_FAST,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_PRE_FAST,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_CHARGING;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_OVER_CHRG,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_OVER_CHRG,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_OVER_CHRG;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_TIMEOUT,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_TIMEOUT,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_TIMEOUT;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_BEL_VPPRE,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_BEL_VPPRE,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_BEL_VPPRE;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_THERMAL_WARNING,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_THERMAL_WARNING,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_THERMAL_WARNING;
  }
  else if(CHECK_BOUNDS(freqs[0],STBC03J_CHG_NTC_WARNING,STBC03J_CHG_FREQ_TH) && CHECK_BOUNDS(freqs[1],STBC03J_CHG_NTC_WARNING,STBC03J_CHG_FREQ_TH))
  {
    ChrgStatus = CHRG_STATUS_NTC_WARNING;
  }
  else
  {
    ChrgStatus = CHRG_STATUS_DISCHARGING;
  }
  
}

/**
* @brief  This method reset the peripherals used to get the current voltage of battery
* @param  None
* @retval COMPONENT_OK in case of success
* @retval COMPONENT_ERROR in case of failures
*/
DrvStatusTypeDef BSP_BatMS_DeInit(void)
{
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    return COMPONENT_ERROR;
  }
  ADC_BLUECOIN_MspDeInit();
  
  return COMPONENT_OK;
  
}

/**
* @brief  This method initializes the peripherals used to get the current voltage of battery
* @param  None
* @retval COMPONENT_OK in case of success
* @retval COMPONENT_ERROR in case of failures
*/
DrvStatusTypeDef BSP_BatMS_Init(void)
{
  /* Variable used to get converted value */
  ADC_ChannelConfTypeDef sConfig;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  BATMS_EN_GPIO_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = BATMS_EN_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_WritePin(BATMS_EN_GPIO_PORT, BATMS_EN_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_Init(BATMS_EN_GPIO_PORT, &GPIO_InitStruct); 
  
   /*##-1- Configure the ADC peripheral #######################################*/
  AdcHandle.Instance          = BATTERY_MONITOR_ADC;
  
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    return COMPONENT_ERROR;
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;   /* Asynchronous clock mode, input ADC clock not divided */
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;            /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = DISABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.EOCSelection          = DISABLE;                        /* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                        /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.NbrOfConversion       = 1;                              /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                        /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 0;                              /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;    /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;  /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                        /* DMA one-shot mode selected (not applied to this example) */
  
  
  ADC_BLUECOIN_MspInit();
  
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    return COMPONENT_ERROR;
  }
  
  /*##-2- Configure ADC regular channel ######################################*/
  sConfig.Channel      = BATTERY_MONITOR_ADC_CHANNEL;                /* Sampled channel number */
  sConfig.Rank         = 1;          /* Rank of sampled channel number BATTERY_MONITOR_ADC_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;    /* Sampling time (number of clock cycles unit) */
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
* @brief  This method enables the sensing of the current voltage of battery
* @param  None
* @retval None
*/
void BSP_BatMS_Enable(void)
{
  HAL_GPIO_WritePin(BATMS_EN_GPIO_PORT, BATMS_EN_GPIO_PIN, GPIO_PIN_SET);
  BatMSEnable=1;
}

/**
* @brief  This method disable the sensing of the current voltage of battery
* @param  None
* @retval None
*/
void BSP_BatMS_Disable(void)
{
  HAL_GPIO_WritePin(BATMS_EN_GPIO_PORT, BATMS_EN_GPIO_PIN, GPIO_PIN_RESET);
  BatMSEnable=0;
}

/**
* @brief  This method enables the sensing of the current voltage of battery
* @param  None
* @retval None
*/
uint8_t BSP_BatMS_isEnable(void)
{
  return BatMSEnable;
}

/**
* @brief  This method gets the current voltage of battery
* @param  volt pointer to destination variable
* @retval COMPONENT_OK in case of success
* @retval COMPONENT_ERROR in case of failures
*/
DrvStatusTypeDef BSP_GetVoltage(uint16_t *volt)
{

  if(!BSP_BatMS_isEnable())
  {
    BSP_BatMS_Enable();
    HAL_Delay(1);
  }
  /*##-3- Start the conversion process #######################################*/
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    return COMPONENT_ERROR;
  }
  
  /*##-4- Wait for the end of conversion #####################################*/
  /*  Before starting a new conversion, you need to check the current state of
  the peripheral; if it’s busy you need to wait for the end of current
  conversion before starting a new one.
  For simplicity reasons, this example is just waiting till the end of the
  conversion, but application may perform other tasks while conversion
  operation is ongoing. */
  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    return COMPONENT_ERROR;
  }
  
  /* Check if the continuous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }
  
  if(BSP_BatMS_isEnable())
  {
    BSP_BatMS_Disable();
  }
  
  
  uint32_t Voltage;
  Voltage = (3000 * (uint32_t)uhADCxConvertedValue) / (4095);  // [0-3V]
  Voltage = ((56+133)*Voltage)/133;   // [0-4.2V]
  
  *volt= Voltage;
  
  return COMPONENT_OK;
}

/**
  * @brief ADC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param None
  * @retval None
  */
void ADC_BLUECOIN_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ADC Periph clock enable */
  BATTERY_MONITOR_ADC_CLK_ENABLE();
  /* Enable GPIO clock ****************************************/
  BATTERY_MONITOR_GPIO_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = BATTERY_MONITOR_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATTERY_MONITOR_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param None
  * @retval None
  */
void ADC_BLUECOIN_MspDeInit(void)
{

  /*##-1- Reset peripherals ##################################################*/
  BATTERY_MONITOR_ADC_FORCE_RESET();
  BATTERY_MONITOR_ADC_RELEASE_RESET();
  
  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(BATTERY_MONITOR_GPIO_PORT, BATTERY_MONITOR_GPIO_PIN);
}

/**
* @brief  Configures LEDs.
* @param  Led: LED to be configured.
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_Init(Led_TypeDef Led)
{

  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}

/**
* @brief  DeInit LEDs.
* @param  Led: LED to be configured.
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @note Led DeInit does not disable the GPIO clock nor disable the Mfx
* @retval None
*/
void BSP_LED_DeInit(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
    LEDx_GPIO_CLK_DISABLE(Led);
}

/**
* @brief  Turns selected LED On.
* @param  Led: LED to be set on
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
* @brief  Turns selected LED Off.
* @param  Led: LED to be set off
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}


/**
* @brief  Toggles the selected LED.
* @param  Led: LED to be toggled
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
*            @arg  LED5
*            @arg  LED6
*            @arg  LED7
*            @arg  LED8
* @retval None
*/
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
* @brief  Configures button GPIO and EXTI Line.
* @param  Button: Button to be configured
*          This parameter can be one of the following values:
*            @arg  BUTTON_1: Push Button 1
*            @arg  BUTTON_2: Push Button 2
* @param  ButtonMode: Button mode
*          This parameter can be one of the following values:
*            @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
*            @arg  BUTTON_MODE_EXTI: Button will be connected to EXTI line
*                                    with interrupt generation capability
* @retval None
*/
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpio_init_structure;
  
  /* Enable the BUTTON clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpio_init_structure.Pin = BUTTON_PIN[Button];
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    gpio_init_structure.Pull = GPIO_PULLDOWN;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);
  }
  
  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    gpio_init_structure.Pin = BUTTON_PIN[Button];
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    if(Button == BUTTON_1)
    {
      gpio_init_structure.Pull = GPIO_PULLDOWN;
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
    }
    else if (Button == BUTTON_2)
    {
      gpio_init_structure.Pull = GPIO_NOPULL;
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
    }
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpio_init_structure);
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
* @brief  Push Button DeInit.
* @param  Button: Button to be configured
*          This parameter can be one of the following values:
*            @arg  BUTTON_WAKEUP: Wakeup Push Button
*            @arg  BUTTON_TAMPER: Tamper Push Button
*            @arg  BUTTON_KEY: Key Push Button
* @note PB DeInit does not disable the GPIO clock
* @retval None
*/
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;
  
  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);
}


/**
* @brief  Returns the selected button state.
* @param  Button: Button to be checked
*          This parameter can be one of the following values:
*            @arg  BUTTON_WAKEUP: Wakeup Push Button
*            @arg  BUTTON_TAMPER: Tamper Push Button
*            @arg  BUTTON_KEY: Key Push Button
* @retval The Button GPIO pin value
*/
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
* @brief  Configures GPIO Mics1234 Source Selector.
* @param  None
* @retval None
*/
void BSP_Mic234_Clock_Selector_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the Mics123SourceSelector Clock */
  MIC234_CLK_EN_GPIO_CLK_ENABLE();
  
  /* Configure the Mics123SourceSelector pin */
  GPIO_InitStruct.Pin = MIC234_CLK_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(MIC234_CLK_EN_GPIO_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(MIC234_CLK_EN_GPIO_PORT, MIC234_CLK_EN_PIN, GPIO_PIN_SET);
}

/**
* @brief  Mics 1234 Source switch.
* @param  Mics1234Source: Specifies the Mics1234 Source to be configured.
*   This parameter can be one of following parameters:
*     @arg ONBOARD
*     @arg EXTERNAL
* @retval None
*/
void BSP_Mic234_Clock_Selector_Set(Mic234Clk_Status_TypeDef Mic234Clk_Status)
{
  if (Mic234Clk_Status == CLK_ENABLE)
  {
    HAL_GPIO_WritePin(MIC234_CLK_EN_GPIO_PORT, MIC234_CLK_EN_PIN, GPIO_PIN_SET);
  }
  if (Mic234Clk_Status == CLK_DISABLE)
  {
    HAL_GPIO_WritePin(MIC234_CLK_EN_GPIO_PORT, MIC234_CLK_EN_PIN, GPIO_PIN_RESET);
  }
}


uint8_t BSP_isBattPlugged()
{
  uint16_t voltage;
  static int i=0;
  static float delta=0.0f;
  static float delta2=0.0f;
  static float mean=0.0f;
  static float M2=0.0f;
  static uint8_t flag =0;
  
  if(i++ <= 100)
  {
    BSP_GetVoltage(&voltage);
    delta= (float)voltage - mean;
    mean= mean +delta/i;
    delta2= (float)voltage -mean;
    M2= M2+ delta*delta2;
  }
  else
  {
    flag= ((M2/(i-1)) < 10000.0f) ? 1 : 0;
    i=0;
    delta=0.0f;
    delta2=0.0f;
    mean=0.0f;
    M2=0.0f;
   
  }
  return flag;
 
}
/*******************************************************************************
BUS OPERATIONS
*******************************************************************************/

/**
* @brief  Configures Bus interface
* @param  None
* @retval COMPONENT_OK in case of success
* @retval COMPONENT_ERROR in case of failures
*/
DrvStatusTypeDef Sensor_IO_Init( void )
{
  if(I2C_ONBOARD_SENSORS_Init() != HAL_OK)
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief  Writes a buffer to the Bus
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  DrvStatusTypeDef ret_val = COMPONENT_OK;
  
  /* Enable I2C multi-bytes Write */
  switch(ctx->who_am_i)
  {
    case  LSM303AGR_ACC_WHO_AM_I:
    case  LSM303AGR_MAG_WHO_AM_I:
    case  LSM6DSM_ACC_GYRO_WHO_AM_I:
    {
      if ( nBytesToWrite > 1 )
        WriteAddr |= 0x80;
        
      if(I2C_ONBOARD_SENSORS_WriteData( pBuffer, ctx->address, WriteAddr, 1, nBytesToWrite ) != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
      return ret_val;
      
    }
    case LPS22HB_WHO_AM_I_VAL:
      /* I2C multi-bytes Read not supported for LPS22HB */
    {
      int i = 0;
      
      for (i = 0; i < nBytesToWrite; i++ )
      {
        /* call I2C_ONBOARD_SENSORS_WriteData Read data bus function */
        if ( I2C_ONBOARD_SENSORS_WriteData( &pBuffer[i], ctx->address, (WriteAddr + i), 1, 1 ) != HAL_OK )
        {
          return COMPONENT_ERROR;
        }
      }
      
      return ret_val;
      
    }
    
    default:
      if(I2C_ONBOARD_SENSORS_WriteData( pBuffer, ctx->address, WriteAddr, 1, nBytesToWrite ) != HAL_OK)
      {
        return COMPONENT_ERROR;
      }
  }
  return COMPONENT_OK;
  
}

/**
 * @brief  Reads a buffer from the Bus
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef Sensor_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  DrvStatusTypeDef ret_val = COMPONENT_OK;
  
  /* Enable I2C multi-bytes read */
  switch(ctx->who_am_i)
  {
    case  LSM303AGR_ACC_WHO_AM_I:
    case  LSM303AGR_MAG_WHO_AM_I:
    case  LSM6DSM_ACC_GYRO_WHO_AM_I:
    {
      if ( nBytesToRead > 1 )
        ReadAddr |= 0x80;
        
      if(I2C_ONBOARD_SENSORS_ReadData( pBuffer, ctx->address, ReadAddr, 1, nBytesToRead ) != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
      
      return ret_val;
    }
    case LPS22HB_WHO_AM_I_VAL:
      /* I2C multi-bytes Read not supported for LPS22HB */
    {
      int i = 0;
      
      for (i = 0; i < nBytesToRead; i++ )
      {
        /* call I2C_ONBOARD_SENSORS_ReadData Read data bus function */
        if ( I2C_ONBOARD_SENSORS_ReadData( &pBuffer[i], ctx->address, (ReadAddr + i), 1, 1 ) != HAL_OK)
        {
          return COMPONENT_ERROR;
        }
      }
      
      return ret_val;
      
    }
    
    default:
      if(I2C_ONBOARD_SENSORS_ReadData( pBuffer, ctx->address, ReadAddr, 1, nBytesToRead ) != HAL_OK)
      {
        return COMPONENT_ERROR;
      }
  }
   return COMPONENT_OK;
  
}

/**
 * @brief  Configures LSM6DSM interrupt lines
 * @retval None
 */
void LSM6DSM_Sensor_IO_ITConfig( void )
{
  GPIO_InitTypeDef GPIO_InitStruc;
  /* Enable INT1 GPIO clock */
  ACC_GYRO_INT1_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStruc.Pin = ACC_GYRO_INT1_PIN;
  GPIO_InitStruc.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruc.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruc.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_GYRO_INT1_GPIO_PORT, &GPIO_InitStruc);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(ACC_GYRO_INT1_EXTI_IRQn, ACC_GYRO_INT1_PRIORITY, 0x00);
  HAL_NVIC_EnableIRQ(ACC_GYRO_INT1_EXTI_IRQn);
  
  
  ACC_GYRO_INT2_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStruc.Pin = ACC_GYRO_INT2_PIN;
  GPIO_InitStruc.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruc.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruc.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_GYRO_INT2_GPIO_PORT, &GPIO_InitStruc);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(ACC_GYRO_INT2_EXTI_IRQn, ACC_GYRO_INT2_PRIORITY, 0x00);
  HAL_NVIC_EnableIRQ(ACC_GYRO_INT2_EXTI_IRQn);
  
}

/**
 * @brief  Configures shutdown pin to switch on the VL53L0x on CoinStation expansion
 * @retval None
 */
void BSP_PROX_Pin_init()
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  __GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
}

/**
 * @brief  Switch on the VL53L0x on CoinStation expansion
 * @retval None
 */
void BSP_PROX_Pin_On()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
 * @brief  Switch off the VL53L0x on CoinStation expansion
 * @retval None
 */
void BSP_PROX_Pin_Off()
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}



/**
 * @brief  Manages error callback by re-initializing Bus.
 * @param  None
 * @retval None
 */
void Sensor_IO_Error( void )
{
  I2C_ONBOARD_SENSORS_Error();
}

/**
 * @brief  Manages error callback by re-initializing I2C.
 * @param  None
 * @retval None
 */
void I2C_ONBOARD_SENSORS_Error(void)
{
  
  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit(&I2C_ONBOARD_SENSORS_Handle);
  
 /* Force the I2C peripheral clock reset */
  I2C_ONBOARD_SENSORS_FORCE_RESET();
  
  /* Release the I2C peripheral clock reset */
  I2C_ONBOARD_SENSORS_RELEASE_RESET();
  /* Re-Initiaize the I2C comunication bus */
  I2C_ONBOARD_SENSORS_Init();
}


/**
  * @brief  I2C error callback fot interrupt mode comunication.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void I2C_ErrorCallback_PROX(I2C_HandleTypeDef *hi2c)
{
  uint32_t er=0x00;
  
  er= HAL_I2C_GetError(hi2c);
  
  if(er == HAL_I2C_ERROR_AF)
  {
    IT_I2C_error_af=1;
  }
}


/**
* @}
*/


/** @defgroup BlueCoin_COMMON_Private_Functions
* @{
*/

/******************************* I2C Routines**********************************/
/**
 * @brief  Configures I2C interface.
 * @param  None
 * @retval None
 */
static HAL_StatusTypeDef I2C_ONBOARD_SENSORS_Init(void)
{
  if(HAL_I2C_GetState(&I2C_ONBOARD_SENSORS_Handle) == HAL_I2C_STATE_RESET)
  {
    /* BlueCoin_I2C_ONBOARD_SENSORS peripheral configuration */
    I2C_ONBOARD_SENSORS_Handle.Init.ClockSpeed = I2C_ONBOARD_SENSORS_SPEED;
    I2C_ONBOARD_SENSORS_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2C_ONBOARD_SENSORS_Handle.Init.OwnAddress1 = 0x33;
    I2C_ONBOARD_SENSORS_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_ONBOARD_SENSORS_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_ONBOARD_SENSORS_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_ONBOARD_SENSORS_Handle.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
    I2C_ONBOARD_SENSORS_Handle.Instance = I2C_ONBOARD_SENSORS;
    /* Init the I2C */
    I2C_ONBOARD_SENSORS_MspInit();
    return HAL_I2C_Init(&I2C_ONBOARD_SENSORS_Handle);
  }
  
  return HAL_OK;
}


/**
 * @brief  Write a value in a register of the device through BUS.
 * @param  pBuffer: Target buffer to be written.
 * @param  DevAddr: Device address on Bus.
 * @param  RegAdd: The target register address to write.
 * @param  RegAddSize: Size of the address.
 * @param  Size: Size of the buffer to be written.
 * @retval HAL status
 */
static HAL_StatusTypeDef I2C_ONBOARD_SENSORS_WriteData(uint8_t* pBuffer, uint8_t DevAddr, uint16_t RegAdd,
                                                       uint8_t RegAddSize, uint16_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  if(RegAddSize == 1)
  {
#ifdef BLUECOIN_I2C_IT
    status = HAL_I2C_Mem_Write_IT(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_8BIT, pBuffer, Size);
#else
    status = HAL_I2C_Mem_Write(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_8BIT, pBuffer, Size, 0x100);
#endif
  }
  else if(RegAddSize == 2)
  {
#ifdef BLUECOIN_I2C_IT
    status = HAL_I2C_Mem_Write_IT(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_16BIT, pBuffer, Size);
#else
    
    status = HAL_I2C_Mem_Write(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_16BIT, pBuffer, Size, 0x100);
#endif
  }
  else
  {
    status = HAL_ERROR;
  }
  
  
#ifdef BLUECOIN_I2C_IT
  while (HAL_I2C_GetState(&I2C_ONBOARD_SENSORS_Handle) != HAL_I2C_STATE_READY)
  {
  }
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C_ONBOARD_SENSORS_Error();
  }
#endif
  return status;
}

/**
 * @brief  Read a register of the device through BUS
 * @param  pBuffer: Target buffer to be read.
 * @param  DevAddr: Device address on Bus.
 * @param  RegAdd: The target register address to read.
 * @param  RegAddSize: Size of the address.
 * @param  Size: Size of the buffer to be read.
 * @retval HAL status
 */
static HAL_StatusTypeDef I2C_ONBOARD_SENSORS_ReadData(uint8_t* pBuffer, uint8_t  DevAddr, uint16_t RegAdd,
    uint8_t RegAddSize, uint16_t Size)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  
  if(RegAddSize == 1)
  {
#ifdef BLUECOIN_I2C_IT
    status = HAL_I2C_Mem_Read_IT(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_8BIT, pBuffer, Size);
#else
    status = HAL_I2C_Mem_Read(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_8BIT, pBuffer, Size, 0x100);
#endif
  }
  else if(RegAddSize == 2)
  {
#ifdef BLUECOIN_I2C_IT
    status = HAL_I2C_Mem_Read_IT(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_16BIT, pBuffer, Size);
#else
    
    status = HAL_I2C_Mem_Read(&I2C_ONBOARD_SENSORS_Handle, DevAddr, RegAdd, I2C_MEMADD_SIZE_16BIT, pBuffer, Size, 0x100);
#endif
    
  }
  else
  {
    status = HAL_ERROR;
  }
  
  
#ifdef BLUECOIN_I2C_IT
  while (HAL_I2C_GetState(&I2C_ONBOARD_SENSORS_Handle) != HAL_I2C_STATE_READY)
  {
    
  }
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2C_ONBOARD_SENSORS_Error();
  }
#endif
  return status;
}
/**
 * @brief  I2C MSP Initialization
 * @param  None
 * @retval None
 */
static void I2C_ONBOARD_SENSORS_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  
  /* Enable I2C GPIO clocks */
  I2C_ONBOARD_SENSORS_SCL_SDA_GPIO_CLK_ENABLE();
  
  /* I2C_ONBOARD_SENSORS SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin = I2C_ONBOARD_SENSORS_SCL_PIN | I2C_ONBOARD_SENSORS_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = I2C_ONBOARD_SENSORS_SCL_SDA_AF;
  HAL_GPIO_Init(I2C_ONBOARD_SENSORS_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
  
  /* Enable the I2C_ONBOARD_SENSORS peripheral clock */
  I2C_ONBOARD_SENSORS_CLK_ENABLE();
  
  /* Force the I2C peripheral clock reset */
  I2C_ONBOARD_SENSORS_FORCE_RESET();
  
  /* Release the I2C peripheral clock reset */
  I2C_ONBOARD_SENSORS_RELEASE_RESET();
  
  /* Enable and set I2C_ONBOARD_SENSORS Interrupt to the highest priority */
  HAL_NVIC_SetPriority(I2C_ONBOARD_SENSORS_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C_ONBOARD_SENSORS_EV_IRQn);
  
  /* Enable and set I2C_ONBOARD_SENSORS Interrupt to the highest priority */
  HAL_NVIC_SetPriority(I2C_ONBOARD_SENSORS_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C_ONBOARD_SENSORS_ER_IRQn);
}


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

/** 
  ******************************************************************************
  * @file    stm32f4xx_nucleo_ika02a1.c
  * @author  JH
  * @version V1.0.0
  * @date    17-Apr-2017
  * @brief   This file contains declarations related to 
  *          GPIO, ADC configuration for X-CUBE-IKA02A1 Expansion Board example software package
  * @endcond
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_nucleo_ika02a1.h"

static uint16_t NUCLEO_ANALOG_PIN[] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_4,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_0};
static GPIO_TypeDef * NUCLEO_ANALOG_GPIO[] = {GPIOA,GPIOA,GPIOA,GPIOB,GPIOC,GPIOC};

static uint32_t NUCLEO_ANALOG_ADC_CHANNEL[] = {ADC_CHANNEL_0,ADC_CHANNEL_1,ADC_CHANNEL_4,ADC_CHANNEL_8,ADC_CHANNEL_11,ADC_CHANNEL_10};

static ADC_TypeDef * NUCLEO_ANALOG_ADC[] = {ADC1,ADC1,ADC1,ADC1,ADC1,ADC1};

uint16_t Get_Gpio_Analog_Value(ADC_HandleTypeDef *hadc,uint8_t input);
static void ADCx_MspInit_Analog(ADC_HandleTypeDef *hadc,uint8_t input);
static void ADCx_Init_Analog(ADC_HandleTypeDef *hadc,uint8_t input);

GPIO_InitTypeDef  GPIO_InitStruct;
ADC_HandleTypeDef hnucleo_adc;
ADC_ChannelConfTypeDef sConfig;

uint8_t status = 1;
uint16_t value;

 /**
   * @brief  Provides value of GPIO pin
   * @return value corresponding to votage on pin
   */
uint16_t Get_ADC_value(uint8_t pin)		
{
      GPIO_InitStruct.Pin = NUCLEO_ANALOG_PIN[pin] ;
      hnucleo_adc.Instance = NUCLEO_ANALOG_ADC[pin];
      return Get_Gpio_Analog_Value(&hnucleo_adc, pin);
}

 /**
   * @brief  Provides value of GPIO pin
   * @return ADC value of pin
   */
uint16_t Get_Gpio_Analog_Value(ADC_HandleTypeDef *hadc,uint8_t input) {
    ADCx_Init_Analog(hadc,input);
   
    /* Select the ADC Channel to be converted */
    sConfig.Channel = NUCLEO_ANALOG_ADC_CHANNEL[input];
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    sConfig.Rank = 1;
    status = HAL_ADC_ConfigChannel(hadc, &sConfig);
    
    status = HAL_ADC_Start(hadc);
    // Wait end of conversion and get value
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
         value = HAL_ADC_GetValue(hadc);
         return value;
    }
    return 0;     
}


 /**
   * @brief  Initializes gpio peripheral, and activates ADC clock
   * @param hadc pointer to ADC handle 
   */
static void ADCx_MspInit_Analog(ADC_HandleTypeDef *hadc,uint8_t input)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
  NUCLEO_ANALOG_GPIO_CLK_ENABLE(input);
  
  /* Configure the selected ADC Channel as analog input */
  GPIO_InitStruct.Pin = NUCLEO_ANALOG_PIN[input] ;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NUCLEO_ANALOG_GPIO[input], &GPIO_InitStruct);
  
  /*** Configure the ADC peripheral ***/ 
  /* Enable ADC clock */
  NUCLEO_ADC_CLK_ENABLE(input); 
}


 /**
   * @brief  Initializes ADC
   * @param hnucleo_Adc pointer to ADC handle 
   */
static void ADCx_Init_Analog(ADC_HandleTypeDef *hadc,uint8_t input)
{
  if(HAL_ADC_GetState(hadc) == HAL_ADC_STATE_RESET)
  {
    /* ADC Config */
    hadc->Instance                   = NUCLEO_ANALOG_ADC[input];
    hadc->Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2; /* (must not exceed 36MHz) */
    hadc->Init.Resolution            = ADC_RESOLUTION12b;
    hadc->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc->Init.ContinuousConvMode    = DISABLE;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.EOCSelection          = EOC_SINGLE_CONV;
    hadc->Init.NbrOfConversion       = 1;
    hadc->Init.DMAContinuousRequests = DISABLE;      
  }
  
  ADCx_MspInit_Analog(hadc,input);
  HAL_ADC_Init(hadc);
}






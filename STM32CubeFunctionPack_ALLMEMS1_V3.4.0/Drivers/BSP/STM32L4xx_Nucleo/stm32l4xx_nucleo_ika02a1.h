/** 
  ******************************************************************************
  * @file    stm32l4xx_nucleo_ika02a1.h
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
#include "stm32l4xx_nucleo.h"

#ifndef __STM32L4XX_NUCLEO_GAS_H__
#define __STM32L4XX_NUCLEO_GAS_H__


//pin definitions for NUCLEO-F401
enum NUCLEO_ANALOG_INPUT {A0=0,A1,A2,A3,A4,A5};

//static uint16_t NUCLEO_ANALOG_PIN[] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_4,GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_0};
//static GPIO_TypeDef * NUCLEO_ANALOG_GPIO[] = {GPIOA,GPIOA,GPIOA,GPIOB,GPIOC,GPIOC};
//
//static uint32_t NUCLEO_ANALOG_ADC_CHANNEL[] = {ADC_CHANNEL_5,ADC_CHANNEL_6,ADC_CHANNEL_9,ADC_CHANNEL_15,ADC_CHANNEL_2,ADC_CHANNEL_1};
//
//static ADC_TypeDef * NUCLEO_ANALOG_ADC[] = {ADC1,ADC1,ADC1,ADC1,ADC1,ADC1};

#define NUCLEO_ANALOG_GPIO_CLK_ENABLE(n)  switch(n)                              \
                   { case 0: __GPIOA_CLK_ENABLE(); break;                        \
                     case 1: __GPIOA_CLK_ENABLE(); break;                        \
                     case 2: __GPIOA_CLK_ENABLE(); break;                        \
                     case 3: __GPIOB_CLK_ENABLE(); break;                        \
                     case 4: __GPIOC_CLK_ENABLE(); break;                        \
                     case 5: __GPIOC_CLK_ENABLE(); break;                        \
                   }
                   
                   
#define NUCLEO_ADC_CLK_ENABLE(n)  switch(n)                              \
                   { case 0: __HAL_RCC_ADC_CLK_ENABLE(); break;                        \
                     case 1: __HAL_RCC_ADC_CLK_ENABLE(); break;                        \
                     case 2: __HAL_RCC_ADC_CLK_ENABLE(); break;                        \
                     case 3: __HAL_RCC_ADC_CLK_ENABLE(); break;                        \
                     case 4: __HAL_RCC_ADC_CLK_ENABLE(); break;                        \
                     case 5: __HAL_RCC_ADC_CLK_ENABLE(); break;                        \
                   }
									 
//function prototypes									 
uint16_t Get_ADC_value(uint8_t pin);							 

#endif // __STM32F4XX_NUCLEO_GAS_H__






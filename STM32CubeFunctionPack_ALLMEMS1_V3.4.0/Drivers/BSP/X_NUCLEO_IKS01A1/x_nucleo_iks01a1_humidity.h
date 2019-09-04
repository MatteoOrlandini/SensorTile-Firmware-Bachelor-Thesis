/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_humidity.h
 * @author  MEMS Application Team
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   This file contains definitions for x_nucleo_iks01a1_humidity.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#ifndef __X_NUCLEO_IKS01A1_HUMIDITY_H
#define __X_NUCLEO_IKS01A1_HUMIDITY_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "HTS221_Driver_HL.h"
#include "x_nucleo_iks01a1.h"


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_HUMIDITY Humidity
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_HUMIDITY_Public_Types Public types
  * @{
  */
#ifdef IKS01A1
typedef enum
{
  HUMIDITY_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  HTS221_H_0                     /* Default on board. */
} HUMIDITY_ID_t;
#else /* IKS01A1 */
  #ifndef DEF_HUMIDITY_ID_t
    #define DEF_HUMIDITY_ID_t
    typedef enum
    {
      HUMIDITY_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
      HTS221_H_0                     /* Default on board. */
    } HUMIDITY_ID_t;
  #endif /* DEF_HUMIDITY_ID_t */
#endif /* IKS01A1 */
/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_HUMIDITY_Public_Defines Public defines
  * @{
  */

#define HUMIDITY_SENSORS_MAX_NUM 1

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_HUMIDITY_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
#ifndef IKS01A1
#undef BSP_HUMIDITY_Init
#define BSP_HUMIDITY_Init BSP_HUMIDITY_Init_IKS01A1
#undef BSP_HUMIDITY_DeInit
#define BSP_HUMIDITY_DeInit BSP_HUMIDITY_DeInit_IKS01A1
#undef BSP_HUMIDITY_Sensor_Enable
#define BSP_HUMIDITY_Sensor_Enable BSP_HUMIDITY_Sensor_Enable_IKS01A1
#undef BSP_HUMIDITY_Sensor_Disable
#define BSP_HUMIDITY_Sensor_Disable BSP_HUMIDITY_Sensor_Disable_IKS01A1
#undef BSP_HUMIDITY_IsInitialized
#define BSP_HUMIDITY_IsInitialized BSP_HUMIDITY_IsInitialized_IKS01A1
#undef BSP_HUMIDITY_IsEnabled
#define BSP_HUMIDITY_IsEnabled BSP_HUMIDITY_IsEnabled_IKS01A1
#undef BSP_HUMIDITY_IsCombo
#define BSP_HUMIDITY_IsCombo BSP_HUMIDITY_IsCombo_IKS01A1
#undef BSP_HUMIDITY_Get_Instance
#define BSP_HUMIDITY_Get_Instance BSP_HUMIDITY_Get_Instance_IKS01A1
#undef BSP_HUMIDITY_Get_WhoAmI
#define BSP_HUMIDITY_Get_WhoAmI BSP_HUMIDITY_Get_WhoAmI_IKS01A1
#undef BSP_HUMIDITY_Check_WhoAmI
#define BSP_HUMIDITY_Check_WhoAmI BSP_HUMIDITY_Check_WhoAmI_IKS01A1
#undef BSP_HUMIDITY_Get_Hum
#define BSP_HUMIDITY_Get_Hum BSP_HUMIDITY_Get_Hum_IKS01A1
#undef BSP_HUMIDITY_Get_ODR
#define BSP_HUMIDITY_Get_ODR BSP_HUMIDITY_Get_ODR_IKS01A1
#undef BSP_HUMIDITY_Set_ODR
#define BSP_HUMIDITY_Set_ODR BSP_HUMIDITY_Set_ODR_IKS01A1
#undef BSP_HUMIDITY_Set_ODR_Value
#define BSP_HUMIDITY_Set_ODR_Value BSP_HUMIDITY_Set_ODR_Value_IKS01A1
#undef BSP_HUMIDITY_Read_Reg
#define BSP_HUMIDITY_Read_Reg BSP_HUMIDITY_Read_Reg_IKS01A1
#undef BSP_HUMIDITY_Write_Reg
#define BSP_HUMIDITY_Write_Reg BSP_HUMIDITY_Write_Reg_IKS01A1
#undef BSP_HUMIDITY_Get_DRDY_Status
#define BSP_HUMIDITY_Get_DRDY_Status BSP_HUMIDITY_Get_DRDY_Status_IKS01A1
#endif /* IKS01A1 */
DrvStatusTypeDef BSP_HUMIDITY_Init( HUMIDITY_ID_t id, void **handle );
DrvStatusTypeDef BSP_HUMIDITY_DeInit( void **handle );
DrvStatusTypeDef BSP_HUMIDITY_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_HUMIDITY_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_HUMIDITY_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_HUMIDITY_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_HUMIDITY_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_HUMIDITY_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_HUMIDITY_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_HUMIDITY_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_HUMIDITY_Get_Hum( void *handle, float *humidity );
DrvStatusTypeDef BSP_HUMIDITY_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_HUMIDITY_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_HUMIDITY_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_HUMIDITY_Read_Reg( void *handle, uint8_t reg, uint8_t *data );
DrvStatusTypeDef BSP_HUMIDITY_Write_Reg( void *handle, uint8_t reg, uint8_t data );
DrvStatusTypeDef BSP_HUMIDITY_Get_DRDY_Status( void *handle, uint8_t *status );

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

#endif /* __X_NUCLEO_IKS01A1_HUMIDITY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

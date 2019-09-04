/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1_temperature.h
 * @author  MEMS Application Team
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   This file contains definitions for x_nucleo_iks01a1_temperature.c
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
#ifndef __X_NUCLEO_IKS01A1_TEMPERATURE_H
#define __X_NUCLEO_IKS01A1_TEMPERATURE_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "HTS221_Driver_HL.h"
#include "LPS25HB_Driver_HL.h"
#include "LPS22HB_Driver_HL.h"
#include "x_nucleo_iks01a1.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_TEMPERATURE Temperature
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_TEMPERATURE_Public_Types Public types
  * @{
  */
#ifdef IKS01A1
typedef enum
{
  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  HTS221_T_0,                    /* HTS221 Default on board. */
  LPS25HB_T_0,                   /* LPS25H/B Default on board. */
  LPS25HB_T_1,                   /* LPS25H/B DIL24 adapter. */
  LPS22HB_T_0                    /* LPS22HB DIL24 adapter. */
} TEMPERATURE_ID_t;
#else /* IKS01A1 */
  #ifndef DEF_TEMPERATURE_ID_t
    #define DEF_TEMPERATURE_ID_t
    typedef enum
    {
      TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
      HTS221_T_0,                    /* HTS221 Default on board (IKS01A1/IKS01A2) */
      LPS25HB_T_0,                   /* LPS25H/B Default on board IKS01A1 */
      LPS25HB_T_1,                   /* LPS25H/B DIL24 adapter on board IKS01A1 */
      LPS22HB_T_0                    /* LPS22HB DIL24 adapter/ Default on board IKS01A2 */
    } TEMPERATURE_ID_t;
  #endif /* DEF_TEMPERATURE_ID_t */
#endif /* IKS01A1 */
/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_TEMPERATURE_Public_Defines Public defines
  * @{
  */

#define TEMPERATURE_SENSORS_MAX_NUM 4

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A1_TEMPERATURE_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
#ifndef IKS01A1
#undef BSP_TEMPERATURE_Init
#define BSP_TEMPERATURE_Init BSP_TEMPERATURE_Init_IKS01A1
#undef BSP_TEMPERATURE_DeInit
#define BSP_TEMPERATURE_DeInit BSP_TEMPERATURE_DeInit_IKS01A1
#undef BSP_TEMPERATURE_Sensor_Enable
#define BSP_TEMPERATURE_Sensor_Enable BSP_TEMPERATURE_Sensor_Enable_IKS01A1
#undef BSP_TEMPERATURE_Sensor_Disable
#define BSP_TEMPERATURE_Sensor_Disable BSP_TEMPERATURE_Sensor_Disable_IKS01A1
#undef BSP_TEMPERATURE_IsInitialized
#define BSP_TEMPERATURE_IsInitialized BSP_TEMPERATURE_IsInitialized_IKS01A1
#undef BSP_TEMPERATURE_IsEnabled
#define BSP_TEMPERATURE_IsEnabled BSP_TEMPERATURE_IsEnabled_IKS01A1
#undef BSP_TEMPERATURE_IsCombo
#define BSP_TEMPERATURE_IsCombo BSP_TEMPERATURE_IsCombo_IKS01A1
#undef BSP_TEMPERATURE_Get_Instance
#define BSP_TEMPERATURE_Get_Instance BSP_TEMPERATURE_Get_Instance_IKS01A1
#undef BSP_TEMPERATURE_Get_WhoAmI
#define BSP_TEMPERATURE_Get_WhoAmI BSP_TEMPERATURE_Get_WhoAmI_IKS01A1
#undef BSP_TEMPERATURE_Check_WhoAmI
#define BSP_TEMPERATURE_Check_WhoAmI BSP_TEMPERATURE_Check_WhoAmI_IKS01A1
#undef BSP_TEMPERATURE_Get_Temp
#define BSP_TEMPERATURE_Get_Temp BSP_TEMPERATURE_Get_Temp_IKS01A1
#undef BSP_TEMPERATURE_Get_ODR
#define BSP_TEMPERATURE_Get_ODR BSP_TEMPERATURE_Get_ODR_IKS01A1
#undef BSP_TEMPERATURE_Set_ODR
#define BSP_TEMPERATURE_Set_ODR BSP_TEMPERATURE_Set_ODR_IKS01A1
#undef BSP_TEMPERATURE_Set_ODR_Value
#define BSP_TEMPERATURE_Set_ODR_Value BSP_TEMPERATURE_Set_ODR_Value_IKS01A1
#undef BSP_TEMPERATURE_Read_Reg
#define BSP_TEMPERATURE_Read_Reg BSP_TEMPERATURE_Read_Reg_IKS01A1
#undef BSP_TEMPERATURE_Write_Reg
#define BSP_TEMPERATURE_Write_Reg BSP_TEMPERATURE_Write_Reg_IKS01A1
#undef BSP_TEMPERATURE_Get_DRDY_Status
#define BSP_TEMPERATURE_Get_DRDY_Status BSP_TEMPERATURE_Get_DRDY_Status_IKS01A1

#undef BSP_TEMPERATURE_FIFO_Get_Full_Status_Ext
#define BSP_TEMPERATURE_FIFO_Get_Full_Status_Ext BSP_TEMPERATURE_FIFO_Get_Full_Status_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Get_Fth_Status_Ext
#define BSP_TEMPERATURE_FIFO_Get_Fth_Status_Ext BSP_TEMPERATURE_FIFO_Get_Fth_Status_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Get_Ovr_Status_Ext
#define BSP_TEMPERATURE_FIFO_Get_Ovr_Status_Ext BSP_TEMPERATURE_FIFO_Get_Ovr_Status_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Get_Data_Ext
#define BSP_TEMPERATURE_FIFO_Get_Data_Ext BSP_TEMPERATURE_FIFO_Get_Data_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Get_Num_Of_Samples_Ext
#define BSP_TEMPERATURE_FIFO_Get_Num_Of_Samples_Ext BSP_TEMPERATURE_FIFO_Get_Num_Of_Samples_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Set_Mode_Ext
#define BSP_TEMPERATURE_FIFO_Set_Mode_Ext BSP_TEMPERATURE_FIFO_Set_Mode_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Set_Interrupt_Ext
#define BSP_TEMPERATURE_FIFO_Set_Interrupt_Ext BSP_TEMPERATURE_FIFO_Set_Interrupt_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Reset_Interrupt_Ext
#define BSP_TEMPERATURE_FIFO_Reset_Interrupt_Ext BSP_TEMPERATURE_FIFO_Reset_Interrupt_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Set_Watermark_Level_Ext
#define BSP_TEMPERATURE_FIFO_Set_Watermark_Level_Ext BSP_TEMPERATURE_FIFO_Set_Watermark_Level_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Stop_On_Fth_Ext
#define BSP_TEMPERATURE_FIFO_Stop_On_Fth_Ext BSP_TEMPERATURE_FIFO_Stop_On_Fth_Ext_IKS01A1
#undef BSP_TEMPERATURE_FIFO_Usage_Ext
#define BSP_TEMPERATURE_FIFO_Usage_Ext BSP_TEMPERATURE_FIFO_Usage_Ext_IKS01A1
#endif /* IKS01A1 */
DrvStatusTypeDef BSP_TEMPERATURE_Init( TEMPERATURE_ID_t id, void **handle );
DrvStatusTypeDef BSP_TEMPERATURE_DeInit( void **handle );
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_TEMPERATURE_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_TEMPERATURE_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_TEMPERATURE_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_TEMPERATURE_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_TEMPERATURE_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_TEMPERATURE_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_TEMPERATURE_Get_Temp( void *handle, float *temperature );
DrvStatusTypeDef BSP_TEMPERATURE_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_TEMPERATURE_Read_Reg( void *handle, uint8_t reg, uint8_t *data );
DrvStatusTypeDef BSP_TEMPERATURE_Write_Reg( void *handle, uint8_t reg, uint8_t data );
DrvStatusTypeDef BSP_TEMPERATURE_Get_DRDY_Status( void *handle, uint8_t *status );

DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Full_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Fth_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Ovr_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Data_Ext( void *handle, float *pressure, float *temperature );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Num_Of_Samples_Ext( void *handle, uint8_t *nSamples );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Mode_Ext( void *handle, uint8_t mode );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Interrupt_Ext( void *handle, uint8_t interrupt );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Reset_Interrupt_Ext( void *handle, uint8_t interrupt );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Watermark_Level_Ext( void *handle, uint8_t watermark );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Stop_On_Fth_Ext( void *handle, uint8_t status );
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Usage_Ext( void *handle, uint8_t status );

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

#endif /* __X_NUCLEO_IKS01A1_TEMPERATURE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

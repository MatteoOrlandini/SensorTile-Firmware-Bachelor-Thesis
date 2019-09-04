/**
 ******************************************************************************
 * @file    x_nucleo_iks01a2_magneto.h
 * @author  MEMS Application Team
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   This file contains definitions for the x_nucleo_iks01a2_magneto.c
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
#ifndef __X_NUCLEO_IKS01A2_MAGNETO_H
#define __X_NUCLEO_IKS01A2_MAGNETO_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "LSM303AGR_MAG_driver_HL.h"
#include "x_nucleo_iks01a2.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2 X_NUCLEO_IKS01A2
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_MAGNETO Magnetometer
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_MAGNETO_Public_Types Public types
  * @{
  */
#ifdef IKS01A2
typedef enum
{
  MAGNETO_SENSORS_AUTO = -1,     /* Always first element and equal to -1 */
  LSM303AGR_M_0                  /* Default on board. */
} MAGNETO_ID_t;
#else /* IKS01A2 */
  #ifndef DEF_MAGNETO_ID_t
    #define DEF_MAGNETO_ID_t
    typedef enum
    {
      MAGNETO_SENSORS_AUTO = -1,     /* Always first element and equal to -1 */
      LIS3MDL_0,                     /* Default on board IKS01A1. */
      LSM303AGR_M_0                  /* Default on board IKS01A2. */
    } MAGNETO_ID_t;
  #endif /* DEF_MAGNETO_ID_t */
#endif /* IKS01A2 */
/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A2_MAGNETO_Public_Defines Public defines
  * @{
  */

#ifdef IKS01A2
  #define MAGNETO_SENSORS_MAX_NUM 1
#else /* IKS01A2 */
  #define MAGNETO_SENSORS_MAX_NUM 2
#endif /* IKS01A2 */

/**
 * @}
 */


/** @addtogroup X_NUCLEO_IKS01A2_MAGNETO_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
#ifndef IKS01A2
#undef BSP_MAGNETO_Init
#define BSP_MAGNETO_Init BSP_MAGNETO_Init_IKS01A2
#undef BSP_MAGNETO_DeInit
#define BSP_MAGNETO_DeInit BSP_MAGNETO_DeInit_IKS01A2
#undef BSP_MAGNETO_Sensor_Enable
#define BSP_MAGNETO_Sensor_Enable BSP_MAGNETO_Sensor_Enable_IKS01A2
#undef BSP_MAGNETO_Sensor_Disable
#define BSP_MAGNETO_Sensor_Disable BSP_MAGNETO_Sensor_Disable_IKS01A2
#undef BSP_MAGNETO_IsInitialized
#define BSP_MAGNETO_IsInitialized BSP_MAGNETO_IsInitialized_IKS01A2
#undef BSP_MAGNETO_IsEnabled
#define BSP_MAGNETO_IsEnabled BSP_MAGNETO_IsEnabled_IKS01A2
#undef BSP_MAGNETO_IsCombo
#define BSP_MAGNETO_IsCombo BSP_MAGNETO_IsCombo_IKS01A2
#undef BSP_MAGNETO_Get_Instance
#define BSP_MAGNETO_Get_Instance BSP_MAGNETO_Get_Instance_IKS01A2
#undef BSP_MAGNETO_Get_WhoAmI
#define BSP_MAGNETO_Get_WhoAmI BSP_MAGNETO_Get_WhoAmI_IKS01A2
#undef BSP_MAGNETO_Check_WhoAmI
#define BSP_MAGNETO_Check_WhoAmI BSP_MAGNETO_Check_WhoAmI_IKS01A2
#undef BSP_MAGNETO_Get_Axes
#define BSP_MAGNETO_Get_Axes BSP_MAGNETO_Get_Axes_IKS01A2
#undef BSP_MAGNETO_Get_AxesRaw
#define BSP_MAGNETO_Get_AxesRaw BSP_MAGNETO_Get_AxesRaw_IKS01A2
#undef BSP_MAGNETO_Get_Sensitivity
#define BSP_MAGNETO_Get_Sensitivity BSP_MAGNETO_Get_Sensitivity_IKS01A2
#undef BSP_MAGNETO_Get_ODR
#define BSP_MAGNETO_Get_ODR BSP_MAGNETO_Get_ODR_IKS01A2
#undef BSP_MAGNETO_Set_ODR
#define BSP_MAGNETO_Set_ODR BSP_MAGNETO_Set_ODR_IKS01A2
#undef BSP_MAGNETO_Set_ODR_Value
#define BSP_MAGNETO_Set_ODR_Value BSP_MAGNETO_Set_ODR_Value_IKS01A2
#undef BSP_MAGNETO_Get_FS
#define BSP_MAGNETO_Get_FS BSP_MAGNETO_Get_FS_IKS01A2
#undef BSP_MAGNETO_Set_FS
#define BSP_MAGNETO_Set_FS BSP_MAGNETO_Set_FS_IKS01A2
#undef BSP_MAGNETO_Set_FS_Value
#define BSP_MAGNETO_Set_FS_Value BSP_MAGNETO_Set_FS_Value_IKS01A2
#undef BSP_MAGNETO_Read_Reg
#define BSP_MAGNETO_Read_Reg BSP_MAGNETO_Read_Reg_IKS01A2
#undef BSP_MAGNETO_Write_Reg
#define BSP_MAGNETO_Write_Reg BSP_MAGNETO_Write_Reg_IKS01A2
#undef BSP_MAGNETO_Get_DRDY_Status
#define BSP_MAGNETO_Get_DRDY_Status BSP_MAGNETO_Get_DRDY_Status_IKS01A2
#endif /* IKS01A2 */
DrvStatusTypeDef BSP_MAGNETO_Init( MAGNETO_ID_t id, void **handle );
DrvStatusTypeDef BSP_MAGNETO_DeInit( void **handle );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_MAGNETO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_MAGNETO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_MAGNETO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_MAGNETO_Get_Axes( void *handle, SensorAxes_t *magnetic_field );
DrvStatusTypeDef BSP_MAGNETO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_MAGNETO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_MAGNETO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_MAGNETO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_MAGNETO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_MAGNETO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_MAGNETO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_MAGNETO_Set_FS_Value( void *handle, float fullScale );
DrvStatusTypeDef BSP_MAGNETO_Read_Reg( void *handle, uint8_t reg, uint8_t *data );
DrvStatusTypeDef BSP_MAGNETO_Write_Reg( void *handle, uint8_t reg, uint8_t data );
DrvStatusTypeDef BSP_MAGNETO_Get_DRDY_Status( void *handle, uint8_t *status );

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

#endif /* __X_NUCLEO_IKS01A2_MAGNETO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

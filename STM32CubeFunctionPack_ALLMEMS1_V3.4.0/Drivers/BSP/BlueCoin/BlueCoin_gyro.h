/**
******************************************************************************
* @file    BlueCoin_gyro.h
* @author  Central Labs
* @version V1.2.0
* @date    23-March-2018
* @brief   This file contains definitions for the BlueCoin_gyro.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLUE_COIN_GYRO_H
#define __BLUE_COIN_GYRO_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "../Components/lsm6dsm/LSM6DSM_ACC_GYRO_driver_HL.h"
#include "BlueCoin.h"


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup BlueCoin BlueCoin
 * @{
 */

/** @addtogroup BlueCoin_GYRO BlueCoin_GYRO
 * @{
 */

/** @addtogroup BlueCoin_GYRO_Public_Types BlueCoin_GYRO_Public types
  * @{
  */

typedef enum
{
  GYRO_SENSORS_AUTO = -1,        /* Always first element and equal to -1 */
  LSM6DS0_G_0,                   /* Default on board. */
  LSM6DSM_G_0                    /* DIL24 adapter. */
} GYRO_ID_t;

/**
 * @}
 */

/** @addtogroup BlueCoin_GYRO_Public_Defines 
  * @{
  */

#define GYRO_SENSORS_MAX_NUM 2

/**
 * @}
 */

/** @addtogroup BlueCoin_GYRO_Public_Function_Prototypes
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_GYRO_Init( GYRO_ID_t id, void **handle );
DrvStatusTypeDef BSP_GYRO_DeInit( void **handle );
DrvStatusTypeDef BSP_GYRO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_GYRO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_GYRO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_GYRO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_GYRO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_GYRO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_GYRO_Get_Axes( void *handle, SensorAxes_t *angular_velocity );
DrvStatusTypeDef BSP_GYRO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_GYRO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_GYRO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_GYRO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_GYRO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_GYRO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_GYRO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_GYRO_Set_FS_Value( void *handle, float fullScale );
DrvStatusTypeDef BSP_GYRO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled );
DrvStatusTypeDef BSP_GYRO_Set_Axes_Status( void *handle, uint8_t *enable_xyz );


DrvStatusTypeDef BSP_GYRO_Read_Reg( void *handle, uint8_t reg, uint8_t *data );
DrvStatusTypeDef BSP_GYRO_Write_Reg( void *handle, uint8_t reg, uint8_t data );
DrvStatusTypeDef BSP_GYRO_Get_DRDY_Status( void *handle, uint8_t *status );

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

#endif /* __BLUE_COIN_GYRO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

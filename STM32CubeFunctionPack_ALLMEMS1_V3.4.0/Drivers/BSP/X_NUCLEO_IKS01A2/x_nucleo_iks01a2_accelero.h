/**
 ******************************************************************************
 * @file    x_nucleo_iks01a2_accelero.h
 * @author  MEMS Application Team
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   This file contains definitions for the x_nucleo_iks01a2_accelero.c
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
#ifndef __X_NUCLEO_IKS01A2_ACCELERO_H
#define __X_NUCLEO_IKS01A2_ACCELERO_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "LSM6DSL_ACC_GYRO_driver_HL.h"
#include "LSM303AGR_ACC_driver_HL.h"
#include "x_nucleo_iks01a2.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2 X_NUCLEO_IKS01A2
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO Accelero
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO_Public_Types Public types
  * @{
  */
#ifdef IKS01A2
typedef enum
{
  ACCELERO_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
  LSM6DSL_X_0,                   /* Default on board. */
  LSM303AGR_X_0                  /* Default on board. */
} ACCELERO_ID_t;
#else /* IKS01A2 */
  #ifndef DEF_ACCELERO_ID_t
    #define DEF_ACCELERO_ID_t
    typedef enum
    {
      ACCELERO_SENSORS_AUTO = -1,    /* Always first element and equal to -1 */
      LSM6DSL_X_0,                    /* Default on board IKS01A2. */
      LSM303AGR_X_0,                  /* Default on board IKS01A2. */
      LSM6DS0_X_0,                    /* Default on board IKS01A1. */
      LSM6DS3_X_0                     /* DIL24 adapter for board IKS01A1. */
    } ACCELERO_ID_t;
  #endif /* DEF_ACCELERO_ID_t */
#endif /* IKS01A2 */
/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO_Public_Defines Public defines
  * @{
  */
#ifdef IKS01A2
  #define ACCELERO_SENSORS_MAX_NUM 2
#else /* IKS01A2 */
  #define ACCELERO_SENSORS_MAX_NUM 4
#endif /* IKS01A2 */

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A2_ACCELERO_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
#ifndef IKS01A2
#undef BSP_ACCELERO_Init
#define BSP_ACCELERO_Init BSP_ACCELERO_Init_IKS01A2
#undef BSP_ACCELERO_DeInit
#define BSP_ACCELERO_DeInit BSP_ACCELERO_DeInit_IKS01A2
#undef BSP_ACCELERO_Sensor_Enable
#define BSP_ACCELERO_Sensor_Enable BSP_ACCELERO_Sensor_Enable_IKS01A2
#undef BSP_ACCELERO_Sensor_Disable
#define BSP_ACCELERO_Sensor_Disable BSP_ACCELERO_Sensor_Disable_IKS01A2
#undef BSP_ACCELERO_IsInitialized
#define BSP_ACCELERO_IsInitialized BSP_ACCELERO_IsInitialized_IKS01A2
#undef BSP_ACCELERO_IsEnabled
#define BSP_ACCELERO_IsEnabled BSP_ACCELERO_IsEnabled_IKS01A2
#undef BSP_ACCELERO_IsCombo
#define BSP_ACCELERO_IsCombo BSP_ACCELERO_IsCombo_IKS01A2
#undef BSP_ACCELERO_Get_Instance
#define BSP_ACCELERO_Get_Instance BSP_ACCELERO_Get_Instance_IKS01A2
#undef BSP_ACCELERO_Get_WhoAmI
#define BSP_ACCELERO_Get_WhoAmI BSP_ACCELERO_Get_WhoAmI_IKS01A2
#undef BSP_ACCELERO_Check_WhoAmI
#define BSP_ACCELERO_Check_WhoAmI BSP_ACCELERO_Check_WhoAmI_IKS01A2
#undef BSP_ACCELERO_Get_Axes
#define BSP_ACCELERO_Get_Axes BSP_ACCELERO_Get_Axes_IKS01A2
#undef BSP_ACCELERO_Get_AxesRaw
#define BSP_ACCELERO_Get_AxesRaw BSP_ACCELERO_Get_AxesRaw_IKS01A2
#undef BSP_ACCELERO_Get_Sensitivity
#define BSP_ACCELERO_Get_Sensitivity BSP_ACCELERO_Get_Sensitivity_IKS01A2
#undef BSP_ACCELERO_Get_ODR
#define BSP_ACCELERO_Get_ODR BSP_ACCELERO_Get_ODR_IKS01A2
#undef BSP_ACCELERO_Set_ODR
#define BSP_ACCELERO_Set_ODR BSP_ACCELERO_Set_ODR_IKS01A2
#undef BSP_ACCELERO_Set_ODR_Value
#define BSP_ACCELERO_Set_ODR_Value BSP_ACCELERO_Set_ODR_Value_IKS01A2
#undef BSP_ACCELERO_Get_FS
#define BSP_ACCELERO_Get_FS BSP_ACCELERO_Get_FS_IKS01A2
#undef BSP_ACCELERO_Set_FS
#define BSP_ACCELERO_Set_FS BSP_ACCELERO_Set_FS_IKS01A2
#undef BSP_ACCELERO_Set_FS_Value
#define BSP_ACCELERO_Set_FS_Value BSP_ACCELERO_Set_FS_Value_IKS01A2
#undef BSP_ACCELERO_Get_Axes_Status
#define BSP_ACCELERO_Get_Axes_Status BSP_ACCELERO_Get_Axes_Status_IKS01A2
#undef BSP_ACCELERO_Set_Axes_Status
#define BSP_ACCELERO_Set_Axes_Status BSP_ACCELERO_Set_Axes_Status_IKS01A2
#undef BSP_ACCELERO_Read_Reg
#define BSP_ACCELERO_Read_Reg BSP_ACCELERO_Read_Reg_IKS01A2
#undef BSP_ACCELERO_Write_Reg
#define BSP_ACCELERO_Write_Reg BSP_ACCELERO_Write_Reg_IKS01A2
#undef BSP_ACCELERO_Get_DRDY_Status
#define BSP_ACCELERO_Get_DRDY_Status BSP_ACCELERO_Get_DRDY_Status_IKS01A2
    
#undef BSP_ACCELERO_Enable_Free_Fall_Detection_Ext
#define BSP_ACCELERO_Enable_Free_Fall_Detection_Ext BSP_ACCELERO_Enable_Free_Fall_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Disable_Free_Fall_Detection_Ext
#define BSP_ACCELERO_Disable_Free_Fall_Detection_Ext BSP_ACCELERO_Disable_Free_Fall_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext
#define BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Free_Fall_Threshold_Ext
#define BSP_ACCELERO_Set_Free_Fall_Threshold_Ext BSP_ACCELERO_Set_Free_Fall_Threshold_Ext_IKS01A2

#undef BSP_ACCELERO_Enable_Pedometer_Ext
#define BSP_ACCELERO_Enable_Pedometer_Ext BSP_ACCELERO_Enable_Pedometer_Ext_IKS01A2
#undef BSP_ACCELERO_Disable_Pedometer_Ext
#define BSP_ACCELERO_Disable_Pedometer_Ext BSP_ACCELERO_Disable_Pedometer_Ext_IKS01A2
#undef BSP_ACCELERO_Get_Pedometer_Status_Ext
#define BSP_ACCELERO_Get_Pedometer_Status_Ext BSP_ACCELERO_Get_Pedometer_Status_Ext_IKS01A2
#undef BSP_ACCELERO_Get_Step_Count_Ext
#define BSP_ACCELERO_Get_Step_Count_Ext BSP_ACCELERO_Get_Step_Count_Ext_IKS01A2
#undef BSP_ACCELERO_Reset_Step_Counter_Ext
#define BSP_ACCELERO_Reset_Step_Counter_Ext BSP_ACCELERO_Reset_Step_Counter_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Pedometer_Threshold_Ext
#define BSP_ACCELERO_Set_Pedometer_Threshold_Ext BSP_ACCELERO_Set_Pedometer_Threshold_Ext_IKS01A2

#undef BSP_ACCELERO_Enable_Tilt_Detection_Ext
#define BSP_ACCELERO_Enable_Tilt_Detection_Ext BSP_ACCELERO_Enable_Tilt_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Disable_Tilt_Detection_Ext
#define BSP_ACCELERO_Disable_Tilt_Detection_Ext BSP_ACCELERO_Disable_Tilt_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Get_Tilt_Detection_Status_Ext
#define BSP_ACCELERO_Get_Tilt_Detection_Status_Ext BSP_ACCELERO_Get_Tilt_Detection_Status_Ext_IKS01A2

#undef BSP_ACCELERO_Enable_Wake_Up_Detection_Ext
#define BSP_ACCELERO_Enable_Wake_Up_Detection_Ext BSP_ACCELERO_Enable_Wake_Up_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Disable_Wake_Up_Detection_Ext
#define BSP_ACCELERO_Disable_Wake_Up_Detection_Ext BSP_ACCELERO_Disable_Wake_Up_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext
#define BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Wake_Up_Threshold_Ext
#define BSP_ACCELERO_Set_Wake_Up_Threshold_Ext BSP_ACCELERO_Set_Wake_Up_Threshold_Ext_IKS01A2

#undef BSP_ACCELERO_Enable_Single_Tap_Detection_Ext
#define BSP_ACCELERO_Enable_Single_Tap_Detection_Ext BSP_ACCELERO_Enable_Single_Tap_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Disable_Single_Tap_Detection_Ext
#define BSP_ACCELERO_Disable_Single_Tap_Detection_Ext BSP_ACCELERO_Disable_Single_Tap_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext
#define BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext_IKS01A2
#undef BSP_ACCELERO_Enable_Double_Tap_Detection_Ext
#define BSP_ACCELERO_Enable_Double_Tap_Detection_Ext BSP_ACCELERO_Enable_Double_Tap_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Disable_Double_Tap_Detection_Ext
#define BSP_ACCELERO_Disable_Double_Tap_Detection_Ext BSP_ACCELERO_Disable_Double_Tap_Detection_Ext_IKS01A2
#undef BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext
#define BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Tap_Threshold_Ext
#define BSP_ACCELERO_Set_Tap_Threshold_Ext BSP_ACCELERO_Set_Tap_Threshold_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Tap_Shock_Time_Ext
#define BSP_ACCELERO_Set_Tap_Shock_Time_Ext BSP_ACCELERO_Set_Tap_Shock_Time_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Tap_Quiet_Time_Ext
#define BSP_ACCELERO_Set_Tap_Quiet_Time_Ext BSP_ACCELERO_Set_Tap_Quiet_Time_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Tap_Duration_Time_Ext
#define BSP_ACCELERO_Set_Tap_Duration_Time_Ext BSP_ACCELERO_Set_Tap_Duration_Time_Ext_IKS01A2

#undef BSP_ACCELERO_Enable_6D_Orientation_Ext
#define BSP_ACCELERO_Enable_6D_Orientation_Ext BSP_ACCELERO_Enable_6D_Orientation_Ext_IKS01A2
#undef BSP_ACCELERO_Disable_6D_Orientation_Ext
#define BSP_ACCELERO_Disable_6D_Orientation_Ext BSP_ACCELERO_Disable_6D_Orientation_Ext_IKS01A2
#undef BSP_ACCELERO_Get_6D_Orientation_Status_Ext
#define BSP_ACCELERO_Get_6D_Orientation_Status_Ext BSP_ACCELERO_Get_6D_Orientation_Status_Ext_IKS01A2
#undef BSP_ACCELERO_Get_6D_Orientation_XL_Ext
#define BSP_ACCELERO_Get_6D_Orientation_XL_Ext BSP_ACCELERO_Get_6D_Orientation_XL_Ext_IKS01A2
#undef BSP_ACCELERO_Get_6D_Orientation_XH_Ext
#define BSP_ACCELERO_Get_6D_Orientation_XH_Ext BSP_ACCELERO_Get_6D_Orientation_XH_Ext_IKS01A2
#undef BSP_ACCELERO_Get_6D_Orientation_YL_Ext
#define BSP_ACCELERO_Get_6D_Orientation_YL_Ext BSP_ACCELERO_Get_6D_Orientation_YL_Ext_IKS01A2
#undef BSP_ACCELERO_Get_6D_Orientation_YH_Ext
#define BSP_ACCELERO_Get_6D_Orientation_YH_Ext BSP_ACCELERO_Get_6D_Orientation_YH_Ext_IKS01A2
#undef BSP_ACCELERO_Get_6D_Orientation_ZL_Ext
#define BSP_ACCELERO_Get_6D_Orientation_ZL_Ext BSP_ACCELERO_Get_6D_Orientation_ZL_Ext_IKS01A2
#undef BSP_ACCELERO_Get_6D_Orientation_ZH_Ext
#define BSP_ACCELERO_Get_6D_Orientation_ZH_Ext BSP_ACCELERO_Get_6D_Orientation_ZH_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Set_ODR_Value_Ext
#define BSP_ACCELERO_FIFO_Set_ODR_Value_Ext BSP_ACCELERO_FIFO_Set_ODR_Value_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Get_Full_Status_Ext
#define BSP_ACCELERO_FIFO_Get_Full_Status_Ext BSP_ACCELERO_FIFO_Get_Full_Status_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Get_Empty_Status_Ext
#define BSP_ACCELERO_FIFO_Get_Empty_Status_Ext BSP_ACCELERO_FIFO_Get_Empty_Status_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Get_Overrun_Status_Ext
#define BSP_ACCELERO_FIFO_Get_Overrun_Status_Ext BSP_ACCELERO_FIFO_Get_Overrun_Status_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Get_Pattern_Ext
#define BSP_ACCELERO_FIFO_Get_Pattern_Ext BSP_ACCELERO_FIFO_Get_Pattern_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Get_Data_Ext
#define BSP_ACCELERO_FIFO_Get_Data_Ext BSP_ACCELERO_FIFO_Get_Data_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Get_Num_Of_Samples_Ext
#define BSP_ACCELERO_FIFO_Get_Num_Of_Samples_Ext BSP_ACCELERO_FIFO_Get_Num_Of_Samples_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Set_Decimation_Ext
#define BSP_ACCELERO_FIFO_Set_Decimation_Ext BSP_ACCELERO_FIFO_Set_Decimation_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Get_Axis_Ext
#define BSP_ACCELERO_FIFO_Get_Axis_Ext BSP_ACCELERO_FIFO_Get_Axis_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Set_Mode_Ext
#define BSP_ACCELERO_FIFO_Set_Mode_Ext BSP_ACCELERO_FIFO_Set_Mode_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Set_INT1_FIFO_Full_Ext
#define BSP_ACCELERO_FIFO_Set_INT1_FIFO_Full_Ext BSP_ACCELERO_FIFO_Set_INT1_FIFO_Full_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext
#define BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext_IKS01A2
#undef BSP_ACCELERO_FIFO_Set_Stop_On_Fth_Ext
#define BSP_ACCELERO_FIFO_Set_Stop_On_Fth_Ext BSP_ACCELERO_FIFO_Set_Stop_On_Fth_Ext_IKS01A2
#undef BSP_ACCELERO_Set_Interrupt_Latch_Ext
#define BSP_ACCELERO_Set_Interrupt_Latch_Ext BSP_ACCELERO_Set_Interrupt_Latch_Ext_IKS01A2
#undef BSP_ACCELERO_Set_SelfTest_Ext
#define BSP_ACCELERO_Set_SelfTest_Ext BSP_ACCELERO_Set_SelfTest_Ext_IKS01A2
   
#undef BSP_ACCELERO_Get_Event_Status_Ext
#define BSP_ACCELERO_Get_Event_Status_Ext BSP_ACCELERO_Get_Event_Status_Ext_IKS01A2
   
#endif /* IKS01A2 */
DrvStatusTypeDef BSP_ACCELERO_Init( ACCELERO_ID_t id, void **handle );
DrvStatusTypeDef BSP_ACCELERO_DeInit( void **handle );
DrvStatusTypeDef BSP_ACCELERO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_ACCELERO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_ACCELERO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_ACCELERO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Axes( void *handle, SensorAxes_t *acceleration );
DrvStatusTypeDef BSP_ACCELERO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_ACCELERO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_ACCELERO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_ACCELERO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_ACCELERO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_ACCELERO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_ACCELERO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_ACCELERO_Set_FS_Value( void *handle, float fullScale );
DrvStatusTypeDef BSP_ACCELERO_Get_Axes_Status( void *handle, uint8_t *xyz_enabled );
DrvStatusTypeDef BSP_ACCELERO_Set_Axes_Status( void *handle, uint8_t *enable_xyz );
DrvStatusTypeDef BSP_ACCELERO_Read_Reg( void *handle, uint8_t reg, uint8_t *data );
DrvStatusTypeDef BSP_ACCELERO_Write_Reg( void *handle, uint8_t reg, uint8_t data );
DrvStatusTypeDef BSP_ACCELERO_Get_DRDY_Status( void *handle, uint8_t *status );

DrvStatusTypeDef BSP_ACCELERO_Enable_Free_Fall_Detection_Ext( void *handle, SensorIntPin_t int_pin );
DrvStatusTypeDef BSP_ACCELERO_Disable_Free_Fall_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Set_Free_Fall_Threshold_Ext( void *handle, uint8_t thr );

DrvStatusTypeDef BSP_ACCELERO_Enable_Pedometer_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Disable_Pedometer_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Pedometer_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Get_Step_Count_Ext( void *handle, uint16_t *step_count );
DrvStatusTypeDef BSP_ACCELERO_Reset_Step_Counter_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Set_Pedometer_Threshold_Ext( void *handle, uint8_t thr );

DrvStatusTypeDef BSP_ACCELERO_Enable_Tilt_Detection_Ext( void *handle, SensorIntPin_t int_pin );
DrvStatusTypeDef BSP_ACCELERO_Disable_Tilt_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Tilt_Detection_Status_Ext( void *handle, uint8_t *status );

DrvStatusTypeDef BSP_ACCELERO_Enable_Wake_Up_Detection_Ext( void *handle, SensorIntPin_t int_pin );
DrvStatusTypeDef BSP_ACCELERO_Disable_Wake_Up_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Set_Wake_Up_Threshold_Ext( void *handle, uint8_t thr );

DrvStatusTypeDef BSP_ACCELERO_Enable_Single_Tap_Detection_Ext( void *handle, SensorIntPin_t int_pin );
DrvStatusTypeDef BSP_ACCELERO_Disable_Single_Tap_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Enable_Double_Tap_Detection_Ext( void *handle, SensorIntPin_t int_pin );
DrvStatusTypeDef BSP_ACCELERO_Disable_Double_Tap_Detection_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Threshold_Ext( void *handle, uint8_t thr );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Shock_Time_Ext( void *handle, uint8_t time );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Quiet_Time_Ext( void *handle, uint8_t time );
DrvStatusTypeDef BSP_ACCELERO_Set_Tap_Duration_Time_Ext( void *handle, uint8_t time );

DrvStatusTypeDef BSP_ACCELERO_Enable_6D_Orientation_Ext( void *handle, SensorIntPin_t int_pin );
DrvStatusTypeDef BSP_ACCELERO_Disable_6D_Orientation_Ext( void *handle );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XL_Ext( void *handle, uint8_t *xl );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_XH_Ext( void *handle, uint8_t *xh );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YL_Ext( void *handle, uint8_t *yl );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_YH_Ext( void *handle, uint8_t *yh );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZL_Ext( void *handle, uint8_t *zl );
DrvStatusTypeDef BSP_ACCELERO_Get_6D_Orientation_ZH_Ext( void *handle, uint8_t *zh );

DrvStatusTypeDef BSP_ACCELERO_Get_Event_Status_Ext( void *handle, ACCELERO_Event_Status_t *status );

DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_ODR_Value_Ext( void *handle, float odr );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Full_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Empty_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Overrun_Status_Ext( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Pattern_Ext( void *handle, uint16_t *pattern );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Data_Ext( void *handle, uint8_t *aData );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Num_Of_Samples_Ext( void *handle, uint16_t *nSamples );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Decimation_Ext( void *handle, uint8_t decimation );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Get_Axis_Ext( void *handle, int32_t *acceleration );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Mode_Ext( void *handle, uint8_t mode );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_INT1_FIFO_Full_Ext( void *handle, uint8_t status );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Watermark_Level_Ext( void *handle, uint16_t watermark );
DrvStatusTypeDef BSP_ACCELERO_FIFO_Set_Stop_On_Fth_Ext( void *handle, uint8_t status );

DrvStatusTypeDef BSP_ACCELERO_Set_Interrupt_Latch_Ext( void *handle, uint8_t status );
DrvStatusTypeDef BSP_ACCELERO_Set_SelfTest_Ext( void *handle, uint8_t status );

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

#endif /* __X_NUCLEO_IKS01A2_ACCELERO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

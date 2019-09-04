/**
  ******************************************************************************
  * @file    MotionFX_Manager.h
  * @author  Central LAB
  * @version V3.4.0
  * @date    26-Apr-2018
  * @brief   This file includes sensor fusion interface functions
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
  
#ifndef _MOTIONFX_MANAGER_H_
#define _MOTIONFX_MANAGER_H_

#include "motion_fx.h"

#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS 500.0f

#define SAMPLE_PERIOD       ((uint8_t)10)   /* [ms] */

/* Exported functions ------------------------------------------------------- */
extern void MotionFX_manager_init(void);
extern void MotionFX_manager_run(SensorAxesRaw_t ACC_Value_Raw,SensorAxes_t GYR_Value,SensorAxes_t MAG_Value);
extern void MotionFX_manager_start_6X(void);
extern void MotionFX_manager_stop_6X(void);
extern void MotionFX_manager_start_9X(void);
extern void MotionFX_manager_stop_9X(void);

extern void MotionFX_manager_MagCal_run(MFX_MagCal_input_t *data_in, MFX_MagCal_output_t *data_out);
extern void MotionFX_manager_MagCal_start(int sampletime);
extern void MotionFX_manager_MagCal_stop(int sampletime);

extern MFX_output_t* MotionFX_manager_getDataOUT(void);
extern MFX_input_t* MotionFX_manager_getDataIN(void);

char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data);
char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data);

#endif //_MOTIONFX_MANAGER_H_

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/


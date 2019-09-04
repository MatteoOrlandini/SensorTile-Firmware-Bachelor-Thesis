/**
 ******************************************************************************
 * @file    DataLog_Manager.c
 * @author  Central LAB
 * @version V3.4.0
 * @date    26-Apr-2018
 * @brief   This file includes Source location interface functions
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
#include "sensor_service.h"
#include "DataLog_Manager.h"
#include "datalog_application.h"

#ifdef ALLMEMS1_ENABLE_SD_CARD_LOGGING

/* Exported Variables -------------------------------------------------------------*/
volatile int  index_buff=0;
volatile uint8_t writeAudio_flag=0;

uint8_t SD_CardLogging_StepHours;
uint8_t SD_CardLogging_StepMinutes;
uint8_t SD_CardLogging_StepSeconds;

uint16_t Audio_OUT_Buff[AUDIO_BUFF_SIZE];


/* Imported Variables -------------------------------------------------------------*/
extern volatile uint8_t SD_Log_Enabled;
extern volatile uint8_t SD_LogAudio_Enabled;

extern volatile uint32_t SD_CardLogging;

/* Code for MotionFX integration - Start Section */
extern SensorAxes_t MAG_Offset;
/* Code for MotionFX integration - End Section */

/* Feature mask that identify the data mens selected for recording*/
extern uint32_t SD_Card_FeaturesMask;
/* Time range for data recording in second */
extern uint32_t SD_Card_StepTime;

/* Enable ShutDown Mode as default with SD card logging features*/
extern uint8_t ShutDownAllowed;

extern uint16_t sdcard_file_counter;

extern uint8_t IsSdMemsRecording;
extern uint8_t IsSdAudioRecording;
extern volatile uint8_t SD_CardNotPresent;

extern uint32_t  DataLogStatus[];

/* RTC handler declared in "main.c" file */
extern RTC_HandleTypeDef RtcHandle;

extern TIM_HandleTypeDef    TimSdRecordingHandle;

extern uint16_t PCM_Buffer[];

/* Private variables -------------------------------------------------------------*/
static char myBuffer[ARRAYSIZE];

/* Private function prototypes ---------------------------------------------------*/
static SensorAxesRaw_t AcceleroRaw_Sensor_Handler_Light(void *handle);
static SensorAxes_t Accelero_Sensor_Handler_Light(void *handle);
static SensorAxes_t Gyro_Sensor_Handler_Light( void *handle );
static SensorAxes_t Magneto_Sensor_Handler_Light( void *handle);
static float Temperature_Sensor_Handler_Light( void *handle );
static float Pressure_Sensor_Handler_Light( void *handle );

#ifndef STM32_BLUECOIN
static float Humidity_Sensor_Handler_Light( void *handle );
#endif /*STM32_BLUECOIN */

/**
  * @brief  Management of the audio data logging
  * @param  None
  * @retval None
  */
void AudioProcess_SD_Recording(void)
{
  uint16_t index = 0;
  static uint16_t OUT_Buff_lvl = 0;
  
  for (index = 0; index < AUDIO_SAMPLING_FREQUENCY/1000 ; index++)
  {
    Audio_OUT_Buff[OUT_Buff_lvl] = PCM_Buffer[index*AUDIO_CHANNELS];
    OUT_Buff_lvl = (OUT_Buff_lvl + 1)%AUDIO_BUFF_SIZE;
  }
  
  //first half
  if(OUT_Buff_lvl == AUDIO_BUFF_SIZE/2)
  {
    index_buff=0;
    writeAudio_flag=1; 
  }    
  //second half
  else if (OUT_Buff_lvl == 0)
  {
    index_buff= AUDIO_BUFF_SIZE;
    writeAudio_flag=1; 
  }    
}

/**
  * @brief  Management of the audio file opening
  * @param  None
  * @retval None
  */
void openFileAudio(void)
{  
  while( (SD_LogAudio_Enabled != 1) &&
         (SD_CardNotPresent != 1) )
  {
    if(DATALOG_SD_LogAudio_Enable())
    {
      SD_LogAudio_Enabled=1;
    }
    else
    {
      DATALOG_SD_LogAudio_Disable();
      //DATALOG_SD_Log_Disable();
      DATALOG_SD_DeInit();
      DATALOG_SD_Init();
    }
    HAL_Delay(100);
  }
}

/**
  * @brief  Management of the audio file closing
  * @param  None
  * @retval None
  */
void closeFileAudio(void)
{
  DATALOG_SD_LogAudio_Disable();
  SD_LogAudio_Enabled=0;
}

/**
  * @brief  Management of the data logging
  * @param  None
  * @retval None
  */
void SD_CardLoggingMemsData(void)
{
  SensorAxesRaw_t AccelerationRaw;
  SensorAxes_t Acceleration;
  SensorAxes_t AngularVelocity;
  SensorAxes_t Magnetometer;
  
  float Temperature_1= 0;
  float Temperature_2= 0;
  float Pressure     = 0;
  float Humidity     = 0;
  
  float Quat[4]= {0.0};
  
  MFX_output_t *MotionFX_Engine_Out;
  
  AccelerationRaw.AXIS_X= 0;
  AccelerationRaw.AXIS_Y= 0;
  AccelerationRaw.AXIS_Z= 0;
  
  Acceleration.AXIS_X= 0;
  Acceleration.AXIS_Y= 0;
  Acceleration.AXIS_Z= 0;
  
  AngularVelocity.AXIS_X= 0;
  AngularVelocity.AXIS_Y= 0;
  AngularVelocity.AXIS_Z= 0;
  
  Magnetometer.AXIS_X= 0;
  Magnetometer.AXIS_Y= 0;
  Magnetometer.AXIS_Z= 0;
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_SENSORFUSION_SHORT)
  {
    AccelerationRaw= AcceleroRaw_Sensor_Handler_Light(TargetBoardFeatures.HandleAccSensor);
    Magnetometer= Magneto_Sensor_Handler_Light(TargetBoardFeatures.HandleMagSensor);
    AngularVelocity= Gyro_Sensor_Handler_Light(TargetBoardFeatures.HandleGyroSensor);
    
    MotionFX_manager_run(AccelerationRaw,AngularVelocity,Magnetometer);

    /* Read the quaternions */
    MotionFX_Engine_Out = MotionFX_manager_getDataOUT();
    
    Quat[0]= MotionFX_Engine_Out->quaternion_9X[0];
    Quat[1]= MotionFX_Engine_Out->quaternion_9X[1];
    Quat[2]= MotionFX_Engine_Out->quaternion_9X[2];
    Quat[3]= MotionFX_Engine_Out->quaternion_9X[3];
    
    AngularVelocity.AXIS_X= 0;
    AngularVelocity.AXIS_Y= 0;
    AngularVelocity.AXIS_Z= 0;
    
    Magnetometer.AXIS_X= 0;
    Magnetometer.AXIS_Y= 0;
    Magnetometer.AXIS_Z= 0;
  }

  if(SD_Card_FeaturesMask & FEATURE_MASK_ACC)
    Acceleration= Accelero_Sensor_Handler_Light(TargetBoardFeatures.HandleAccSensor);
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_GRYO)
    AngularVelocity= Gyro_Sensor_Handler_Light(TargetBoardFeatures.HandleGyroSensor);
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_MAG)
    Magnetometer= Magneto_Sensor_Handler_Light(TargetBoardFeatures.HandleMagSensor);
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_PRESS)
    Pressure= Pressure_Sensor_Handler_Light(TargetBoardFeatures.HandlePressSensor);
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_TEMP1)
    Temperature_1= Temperature_Sensor_Handler_Light(TargetBoardFeatures.HandleTempSensors[0]);
  
  if(SD_Card_FeaturesMask & FEATURE_MASK_TEMP2)
    Temperature_2= Temperature_Sensor_Handler_Light(TargetBoardFeatures.HandleTempSensors[1]);

#ifndef STM32_BLUECOIN  
  if(SD_Card_FeaturesMask & FEATURE_MASK_HUM)
    Humidity= Humidity_Sensor_Handler_Light(TargetBoardFeatures.HandleHumSensor);
#endif /* STM32_BLUECOIN */ 
  
  openFile(OPEN_FILE_FOR_APP);
  saveData(myBuffer, Quat, Acceleration, AngularVelocity, Magnetometer, Pressure, Temperature_1, Temperature_2, Humidity, FREQUENCY);
  closeFile();
}

/**
  * @brief  Management of the file opening
  * @param  None
  * @retval None
  */
void openFile(uint8_t AccessControl)
{  
  while( (SD_Log_Enabled != 1) &&
         (SD_CardNotPresent != 1) )
  {
    if(DATALOG_SD_Log_Enable(AccessControl))
    {
      SD_Log_Enabled=1;
    }
    else
    {
      DATALOG_SD_Log_Disable();
      DATALOG_SD_DeInit();
      DATALOG_SD_Init();
    }
    HAL_Delay(100);
  }
}

/**
  * @brief  Management of the file closing
  * @param  None
  * @retval None
  */
void closeFile(void)
{
  DATALOG_SD_Log_Disable();
  SD_Log_Enabled=0;
}

/**
 * @brief  Management of the start of the SD Card data logging
 * @param  None
 * @retval None
 */
void SD_CardLoggingMemsStart(void)
{
  if(!IsSdAudioRecording)
  {
    if(!IsSdMemsRecording)
    {
      LedOffTargetPlatform();
      openFile(CREATE_FILE_FOR_WRITE);
     
      if(!SD_CardNotPresent)
      {
        IsSdMemsRecording= 1;
        closeFile();
        
        DataLogStatus[0]= (uint32_t)0x12345678;
        DataLogStatus[1]= (uint32_t)0x00000001;
        DataLogStatus[2]= SD_Card_FeaturesMask;
        DataLogStatus[3]= SD_Card_StepTime;
        DataLogStatus[4]= (uint32_t)sdcard_file_counter;
        MDM_SaveGMD(GMD_DATA_LOG_STATUS,(void *)&DataLogStatus);
        NecessityToSaveMetaDataManager=1;
        
        if(SD_Card_StepTime > 0)
        {
          #ifdef STM32_SENSORTILE
          if(SD_Card_StepTime < (RANGE_TIME_WITHOUT_CONNECTED/1000))
          #endif /* STM32_SENSORTILE */
          {
            ShutDownAllowed= 0;
          }
        }
        else
        {
          /* Start the TIM Base generation in interrupt mode */
          if(HAL_TIM_Base_Start_IT(&TimSdRecordingHandle) != HAL_OK){
            /* Starting Error */
            Error_Handler();
          }
          
          ShutDownAllowed= 0;
        }
        
        BytesToWrite =sprintf((char *)BufferToWrite,"Start Data Log MEMS Recording\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        BytesToWrite =sprintf((char *)BufferToWrite,"SD Card not present\r\n");
        Term_Update(BufferToWrite,BytesToWrite);
        SD_CardNotPresent= 0;
        BSP_LED_Init(LED1);  
        BSP_LED_Off(LED1);
      }
    }
    else
    {
      BytesToWrite =sprintf((char *)BufferToWrite,"Data Log MEMS is already started\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
    }
  }
  else
  {
    BytesToWrite =sprintf((char *)BufferToWrite,"Data Log Audio is already started\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
    BytesToWrite =sprintf((char *)BufferToWrite,"It is need stopped the Data Log Audio before\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  }
}

/**
 * @brief  Management of the stop of the SD Card data logging
 * @param  None
 * @retval None
 */
void SD_CardLoggingMemsStop(void)
{
  if(IsSdMemsRecording)
  {
    BytesToWrite =sprintf((char *)BufferToWrite,"Stop Data Log MEMS Recording\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
    
    /* Deactivate alarm if it was activated */
    HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
    
    /* Stop Timer For Sd Card Recording if it was started */
    if(HAL_TIM_Base_Stop_IT(&TimSdRecordingHandle) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    ShutDownAllowed= 1;
    SD_CardLogging=0;
    
    IsSdMemsRecording= 0;
//    BSP_LED_Init(LED1);  
//    LedOffTargetPlatform();
    
    DataLogStatus[0]= (uint32_t)0x12345678;
    DataLogStatus[1]= (uint32_t)0x00000000;
    DataLogStatus[2]= SD_Card_FeaturesMask;
    DataLogStatus[3]= SD_Card_StepTime;
    DataLogStatus[4]= (uint32_t)sdcard_file_counter;
    MDM_SaveGMD(GMD_DATA_LOG_STATUS,(void *)&DataLogStatus);
    NecessityToSaveMetaDataManager=1;
  }
  else
  {
    BytesToWrite =sprintf((char *)BufferToWrite,"None Data Log MEMS\r\n");
    Term_Update(BufferToWrite,BytesToWrite);          
  }
}

/**
 * @brief  Handles the accelerometer Raw axes data, fast
 * @param  handle the device handle
 * @retval None
 */
static SensorAxesRaw_t AcceleroRaw_Sensor_Handler_Light(void *handle)
{
  uint8_t id;
  SensorAxesRaw_t accelerationRaw;
  uint8_t status;
  
  accelerationRaw.AXIS_X = 0;
  accelerationRaw.AXIS_Y = 0;
  accelerationRaw.AXIS_Z = 0;

  BSP_ACCELERO_Get_Instance(handle, &id);

  BSP_ACCELERO_IsInitialized(handle, &status);
  
  accelerationRaw.AXIS_X = 0;
  accelerationRaw.AXIS_Y = 0;
  accelerationRaw.AXIS_Z = 0;

  if (status == 1)
  {
    if (BSP_ACCELERO_Get_AxesRaw(handle, &accelerationRaw) == COMPONENT_ERROR)
    {
      accelerationRaw.AXIS_X = 0;
      accelerationRaw.AXIS_Y = 0;
      accelerationRaw.AXIS_Z = 0;
    }    
  }
  
  return accelerationRaw;
}
        
/**
 * @brief  Handles the accelerometer axes data, fast
 * @param  handle the device handle
 * @retval None
 */
static SensorAxes_t Accelero_Sensor_Handler_Light(void *handle)
{
  uint8_t id;
  SensorAxes_t acceleration;
  uint8_t status;
  
  acceleration.AXIS_X = 0;
  acceleration.AXIS_Y = 0;
  acceleration.AXIS_Z = 0;

  BSP_ACCELERO_Get_Instance(handle, &id);

  BSP_ACCELERO_IsInitialized(handle, &status);
  
  acceleration.AXIS_X = 0;
  acceleration.AXIS_Y = 0;
  acceleration.AXIS_Z = 0;

  if (status == 1)
  {
    if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR)
    {
      acceleration.AXIS_X = 0;
      acceleration.AXIS_Y = 0;
      acceleration.AXIS_Z = 0;
    }    
  }
  
  return acceleration;
}

/**
* @brief  Handles the gyroscope axes data getting/sending
* @param  handle the device handle
* @retval None
*/
static SensorAxes_t Gyro_Sensor_Handler_Light( void *handle )
{
  uint8_t id;
  SensorAxes_t angular_velocity;
  uint8_t status;
  
  BSP_GYRO_Get_Instance( handle, &id );
  
  BSP_GYRO_IsInitialized( handle, &status );
  
  angular_velocity.AXIS_X = 0;
  angular_velocity.AXIS_Y = 0;
  angular_velocity.AXIS_Z = 0;
  
  if ( status == 1 )
  {
    if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
    {
      angular_velocity.AXIS_X = 0;
      angular_velocity.AXIS_Y = 0;
      angular_velocity.AXIS_Z = 0;
    }    
  }
  
  return angular_velocity;
}

/**
* @brief  Handles the magneto axes data getting/sending
* @param  handle the device handle
* @retval None
*/
static SensorAxes_t Magneto_Sensor_Handler_Light( void *handle )
{
  uint8_t id;
  SensorAxes_t magnetic_field;
  uint8_t status;
  
  BSP_MAGNETO_Get_Instance( handle, &id );
  
  BSP_MAGNETO_IsInitialized( handle, &status );
  
  magnetic_field.AXIS_X = 0;
  magnetic_field.AXIS_Y = 0;
  magnetic_field.AXIS_Z = 0;
  
  if ( status == 1 )
  {
    if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_OK )
    {
      /* Apply Magneto calibration */
      magnetic_field.AXIS_X = magnetic_field.AXIS_X - MAG_Offset.AXIS_X;
      magnetic_field.AXIS_Y = magnetic_field.AXIS_Y - MAG_Offset.AXIS_Y;
      magnetic_field.AXIS_Z = magnetic_field.AXIS_Z - MAG_Offset.AXIS_Z;
    }
    else
    {
      magnetic_field.AXIS_X = 0;
      magnetic_field.AXIS_Y = 0;
      magnetic_field.AXIS_Z = 0;
    }
  }
  
  return magnetic_field; 
}

/**
* @brief  Handles the temperature data getting/sending
* @param  handle the device handle
* @retval None
*/
static float Temperature_Sensor_Handler_Light( void *handle )
{
  uint8_t id;
  float temperature = 0.0;
  uint8_t status;
  
  BSP_TEMPERATURE_Get_Instance( handle, &id );
  
  BSP_TEMPERATURE_IsInitialized( handle, &status );
  
  if ( status == 1 )
  {
    if ( BSP_TEMPERATURE_Get_Temp( handle, &temperature ) == COMPONENT_ERROR )
    {
      temperature = 0.0f;
    }    
  }
  
  return temperature;
}

/**
* @brief  Handles the pressure sensor data getting/sending
* @param  handle the device handle
* @retval None
*/
static float Pressure_Sensor_Handler_Light( void *handle )
{
  uint8_t id;
  float pressure;
  uint8_t status;
  
  BSP_PRESSURE_Get_Instance( handle, &id );
  
  BSP_PRESSURE_IsInitialized( handle, &status );
  
  pressure = 0.0;
  
  if( status == 1 )
  {
    if ( BSP_PRESSURE_Get_Press( handle, &pressure ) == COMPONENT_ERROR )
    {
      pressure = 0.0f;
    }    
  }
  
  return pressure;
}

#ifndef STM32_BLUECOIN
/**
* @brief  Handles the humidity data getting/sending
* @param  handle the device handle
* @retval None
*/
static float Humidity_Sensor_Handler_Light( void *handle )
{
  uint8_t id;
  float humidity;
  uint8_t status;
  
  BSP_HUMIDITY_Get_Instance( handle, &id );
  
  humidity = 0.0f;
  
  BSP_HUMIDITY_IsInitialized( handle, &status );
  
  if ( status == 1 )
  {
    if ( BSP_HUMIDITY_Get_Hum( handle, &humidity ) == COMPONENT_ERROR )
    {
      humidity = 0.0f;
    }    
  }
  return humidity;
}
#endif /* STM32_BLUECOIN */

#endif /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

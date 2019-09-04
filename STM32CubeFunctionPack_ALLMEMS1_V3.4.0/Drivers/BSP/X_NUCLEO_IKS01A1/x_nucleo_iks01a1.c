/**
 ******************************************************************************
 * @file    x_nucleo_iks01a1.c
 * @author  MEMS Application Team
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   This file provides X_NUCLEO_IKS01A1 MEMS shield board specific functions
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

/* Includes ------------------------------------------------------------------*/

#include "x_nucleo_iks01a1.h"

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_I2C.h"
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#include "stm32l4xx_I2C.h"
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
#include "stm32l0xx_I2C.h"
#endif /* USE_STM32L0XX_NUCLEO */


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1 X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_IO IO
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1_IO_Private_Variables Private variables
 * @{
 */
#ifdef IKS01A1
static uint32_t I2C_EXPBD_Timeout = NUCLEO_I2C_EXPBD_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
#endif /* IKS01A1 */
#define I2C_EXPBD_Handle I2CHandle
#define I2C_EXPBD_Init I2C_Global_Init
#define I2C_EXPBD_Error I2C_Global_Error
//static I2C_HandleTypeDef I2C_EXPBD_Handle;

/**
 * @}
 */
#ifdef IKS01A1
/* Link function for sensor peripheral */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

static uint8_t I2C_EXPBD_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t I2C_EXPBD_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
#endif /* IKS01A1 */
/** @addtogroup X_NUCLEO_IKS01A1_IO_Public_Functions Public functions
 * @{
 */

/**
 * @brief  Configures sensor I2C interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_Init( void )
{

  if ( I2C_EXPBD_Init() )
  {
    return COMPONENT_ERROR;
  }
  else
  {
    return COMPONENT_OK;
  }
}



/**
 * @brief  Configures sensor interrupts interface for LSM6DS0 sensor).
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DS0_Sensor_IO_ITConfig( void )
{

  /*Not implemented yet*/

  return COMPONENT_OK;
}



/**
 * @brief  Configures sensor interrupts interface for LSM6DS3 sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DS3_Sensor_IO_ITConfig( void )
{

  /* At the moment this feature is only implemented for LSM6DS3 */
  GPIO_InitTypeDef GPIO_InitStructureInt1;
#if 0
  GPIO_InitTypeDef GPIO_InitStructureInt2;
#endif
  /* Enable INT1 GPIO clock */
  M_INT1_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = M_INT1_PIN;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(M_INT1_GPIO_PORT, &GPIO_InitStructureInt1);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(M_INT1_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(M_INT1_EXTI_IRQn);

#if 0
  /* Enable INT2 GPIO clock */
  M_INT2_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt2.Pin = M_INT2_PIN;
  GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt2.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(M_INT2_GPIO_PORT, &GPIO_InitStructureInt2);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(M_INT2_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(M_INT2_EXTI_IRQn);
#endif
  return COMPONENT_OK;
}



/**
 * @brief  Configures sensor interrupts interface for LPS22HB sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LPS22HB_Sensor_IO_ITConfig( void )
{

  /* At the moment this feature is only implemented for LPS22HB */
  GPIO_InitTypeDef GPIO_InitStructureInt1;
  /* Enable INT1 GPIO clock */
  M_INT1_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = M_INT1_PIN;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(M_INT1_GPIO_PORT, &GPIO_InitStructureInt1);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(M_INT1_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(M_INT1_EXTI_IRQn);

  return COMPONENT_OK;
}

/**
 * @}
 */


/** @addtogroup X_NUCLEO_IKS01A1_IO_Private_Functions Private functions
 * @{
 */
#ifdef IKS01A1
/******************************* Link functions *******************************/


/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  switch(ctx->who_am_i)
  {
    case IKS01A1_LPS25HB_WHO_AM_I:
    case IKS01A1_LIS3MDL_WHO_AM_I:
    case IKS01A1_HTS221_WHO_AM_I:
    {
      if ( nBytesToWrite > 1 ) WriteAddr |= 0x80;  /* Enable I2C multi-bytes Write */

      /* call I2C_EXPBD Write data bus function */
      if ( I2C_EXPBD_WriteData( ctx->address, WriteAddr, pBuffer, nBytesToWrite ) )
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
    case IKS01A1_LPS22HB_WHO_AM_I:
    {
      /* I2C multi-bytes Write not supported for LPS22HB */
      int i = 0;

      for (i = 0; i < nBytesToWrite; i++ )
      {
        /* call I2C_EXPBD Write data bus function */
        if ( I2C_EXPBD_WriteData( ctx->address, (WriteAddr + i), &pBuffer[i], 1 ) )
        {
          return 1;
        }
      }

      return 0;
    }
    case IKS01A1_LSM6DS0_WHO_AM_I:
    case IKS01A1_LSM6DS3_WHO_AM_I:
    default:
    {
      /* call I2C_EXPBD Write data bus function */
      if ( I2C_EXPBD_WriteData( ctx->address, WriteAddr, pBuffer, nBytesToWrite ) )
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
  }
}



/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  switch(ctx->who_am_i)
  {
    case IKS01A1_LPS25HB_WHO_AM_I:
    case IKS01A1_LIS3MDL_WHO_AM_I:
    case IKS01A1_HTS221_WHO_AM_I:
    {
      if ( nBytesToRead > 1 ) ReadAddr |= 0x80; /* Enable I2C multi-bytes Read */

      /* call I2C_EXPBD Read data bus function */
      if ( I2C_EXPBD_ReadData( ctx->address, ReadAddr, pBuffer, nBytesToRead ) )
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
    case IKS01A1_LPS22HB_WHO_AM_I:
    {
      /* I2C multi-bytes Read not supported for LPS22HB */
      int i = 0;

      for (i = 0; i < nBytesToRead; i++ )
      {
        /* call I2C_EXPBD Read data bus function */
        if ( I2C_EXPBD_ReadData( ctx->address, (ReadAddr + i), &pBuffer[i], 1 ) )
        {
          return 1;
        }
      }

      return 0;
    }
    case IKS01A1_LSM6DS0_WHO_AM_I:
    case IKS01A1_LSM6DS3_WHO_AM_I:
    default:
    {
      /* call I2C_EXPBD Read data bus function */
      if ( I2C_EXPBD_ReadData( ctx->address, ReadAddr, pBuffer, nBytesToRead ) )
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
  }
}



/******************************* I2C Routines *********************************/

/**
 * @brief  Write data to the register of the device through BUS
 * @param  Addr Device address on BUS
 * @param  Reg The target register address to be written
 * @param  pBuffer The data to be written
 * @param  Size Number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t I2C_EXPBD_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                              I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    I2C_EXPBD_Error( Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}



/**
 * @brief  Read a register of the device through BUS
 * @param  Addr Device address on BUS
 * @param  Reg The target register address to read
 * @param  pBuffer The data to be read
 * @param  Size Number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t I2C_EXPBD_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    I2C_EXPBD_Error( Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}
#endif /* IKS0A1 */
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

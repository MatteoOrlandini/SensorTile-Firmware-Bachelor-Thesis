 /**
 ******************************************************************************
 * @file    DataLog_Manager.h
 * @author  Central Lab
 * @version V3.4.0
 * @date    26-Apr-2018
 * @brief   Header for DataLog_Manager.c
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DATA_LOG_MANAGER_H_
#define _DATA_LOG_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define WRITE_EACH 8 //ORIGiNAL
#define WRITE_EACH 32
#define AUDIO_BUFF_SIZE (AUDIO_SAMPLING_FREQUENCY / 1000 * 1 * (WRITE_EACH*2))
  
#define ARRAYSIZE 10816//9152//8192

#define BUFF_SIZE 104
#define FREQUENCY BUFF_SIZE
  
#define CREATE_FILE_FOR_WRITE 0
#define OPEN_FILE_FOR_APP     1

/* Exported Functions Prototypes ---------------------------------------------*/
extern void AudioProcess_SD_Recording(void);
extern void openFileAudio(void);
extern void closeFileAudio(void);

extern void SD_CardLoggingMemsData(void);
extern void openFile(uint8_t AccessControl);
extern void closeFile(void);

extern void SD_CardLoggingMemsStart(void);
extern void SD_CardLoggingMemsStop(void);

#ifdef __cplusplus
}
#endif

#endif /* _DATA_LOG_MANAGER_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


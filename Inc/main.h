/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Button0_Pin GPIO_PIN_13
#define Button0_GPIO_Port GPIOC
#define MOT4_A_Pin GPIO_PIN_0
#define MOT4_A_GPIO_Port GPIOH
#define MOT3_A_Pin GPIO_PIN_1
#define MOT3_A_GPIO_Port GPIOH
#define MOT1_B_Pin GPIO_PIN_0
#define MOT1_B_GPIO_Port GPIOC
#define MOT2_B_Pin GPIO_PIN_1
#define MOT2_B_GPIO_Port GPIOC
#define MOT2_A_Pin GPIO_PIN_2
#define MOT2_A_GPIO_Port GPIOC
#define MOT1_A_Pin GPIO_PIN_3
#define MOT1_A_GPIO_Port GPIOC
#define MOT4_B_Pin GPIO_PIN_1
#define MOT4_B_GPIO_Port GPIOA
#define MOT3_B_Pin GPIO_PIN_4
#define MOT3_B_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_10
#define PWM4_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_3
#define PWM1_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_4
#define PWM3_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_5
#define PWM2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

void printf_IT(char* str);



/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

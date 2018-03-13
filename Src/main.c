/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "control.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "string.h"
#include "fonts.h"
#include "ssd1306.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
float M_PI=3.14f;
float angle[3], angle_offset[3];			//Yaw,Pitch,Roll


uint8_t tick_tock = 0;
uint8_t run=0,run10=0;
uint8_t rxBuffer[256]="";
uint8_t positionData[256]="";


float pos_x,pos_y,tar_x,tar_y=0;
float direction = 0;
float target_direction = 0;
uint8_t turn_direction = 0; //1-left 2-right
uint8_t go_forward = 0; //1-foreward 2-backward
float distance = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void printf_IT(char* str)
{
	char tmpstr[256] = "";
	strcpy(tmpstr, str);
	HAL_UART_Transmit_IT(&huart2, tmpstr, strlen(tmpstr));
}

void printf_DMA(char* tmpstr)
{
	
	HAL_UART_Transmit_DMA(&huart2, tmpstr, strlen(tmpstr));
}


void update_DMP(void)
{
	mpuIntStatus = MPUgetIntStatus();
		if (mpuIntStatus & 0x10){
			MPUresetFIFO();
		}
		else if(mpuIntStatus & 0x02){
			MPUgetFIFOBytes(fifoBuffer, packetSize);
			MPUdmpGetQuaternion(&q, fifoBuffer);
			MPUdmpGetEuler(euler, &q);
			angle[0] = euler[0]* 180/M_PI; 
			angle[1] = euler[2]* 180/M_PI; 
			angle[2] = euler[1]* 180/M_PI;
		}
			
}

int fabs_min_index(float a[],int size)
{
	int i,min=0;
	for(i=1;i<size;i++)
		if(fabs(a[i])<fabs(a[min]))
			min=i;
	return min;
}

int get_turn_direction(float target_direction,float direction)
{
	uint8_t turn_direction=0;
	target_direction += 3.1415926f*4.0f;
	float delta_direction_array[5];
	for(int index=0;index<5;index++)
	{
		delta_direction_array[index] = target_direction - direction;
		target_direction -= 3.1415926f*2.0f;
	}
	int idx=0;
	idx = fabs_min_index(delta_direction_array, 5);
	if(delta_direction_array[idx]>0.35f)
	{
		turn_direction = 2;
	}
	else if(delta_direction_array[idx] <-0.35f)
	{
		turn_direction = 1;
	}
	else
	{
		turn_direction = 0;
	}
	return turn_direction;
}

float get_direction(float pos_x,float pos_y,float tar_x,float tar_y)
{
	return atan((tar_y-pos_y)/(tar_x-pos_x));
}

void convert_position_data(uint8_t* positionData)
{
	sscanf(positionData,"A%f,%f,%f,%fF",&pos_x,&pos_y,&tar_x,&tar_y);
}
float get_target_distance(float pos_x,float pos_y,float tar_x,float tar_y)
{
	return sqrt(pow(tar_x-pos_x,2)+pow(tar_y-pos_y,2));
}

uint8_t get_go_forward_action(float distance)
{
	if(distance>10)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	char str[256] = "";
	
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM1_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
	
	
	TIM2->CCR2 = 50;
	TIM2->CCR3 = 50;
	TIM3->CCR1 = 50;
	TIM3->CCR2 = 50;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	
	MPU6050(0xD0);
	MPUinitialize();
	if( MPUtestConnection()== SUCCESS) {
		printf("MPUConnect OK!\n");
	}
	else
	{
		printf("***********Not Success\n");
	}
	MPUdmpInitialize();
	MPUsetDMPEnabled(true);
	mpuIntStatus = MPUgetIntStatus();
	packetSize = MPUdmpGetFIFOPacketSize();
	//ssd1306_Init();
	HAL_Delay(300);
	//ssd1306_Fill(White);
	//ssd1306_UpdateScreen();
	
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_UART_Receive_IT(&huart2, rxBuffer, 256);
	HAL_Delay(500);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(run){
			run=0;
			if(!HAL_GPIO_ReadPin(Button0_GPIO_Port, Button0_Pin)){
				for(int dimension = 0;dimension < 3; dimension++){
					angle_offset[dimension] = angle[dimension];
				}
			}
			
			convert_position_data(positionData);	
			distance = get_target_distance(pos_x,pos_y,tar_x,tar_y);
			direction = (angle[0]-angle_offset[0])/180.0*3.14159;
			target_direction = get_direction(pos_x,pos_y,tar_x,tar_y);
			turn_direction = get_turn_direction(target_direction,direction);
			go_forward = get_go_forward_action(distance);
			
			
			
			
			

		}
		//HAL_Delay(100);
		if(run10){
			run10=0;
			//sprintf(str,"C%.4fF\n",direction);
			sprintf(str,"Direction:%.2f,TargetDirection:%.2f,TurnDirection:%d,Distance:%.2f,GoForward:%d\r\n",
						direction,target_direction,turn_direction,distance,go_forward);
			//sprintf(str,"Yaw: %.2f  Pitch: %.2f Roll: %.2f, PosX: %.2f,PosY: %.2f,TarX: %.2f,TarY: %.2f\n",angle[0]-angle_offset[0],angle[1]-angle_offset[1],angle[2]-angle_offset[2],pos_x,pos_y,tar_x,tar_y);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			printf_DMA(str);
			
		}
		
		update_DMP();
		
	  
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim11.Instance)
	{
		run=1;
		tick_tock += 1;
		if(tick_tock == 9)
		{
			tick_tock = 0;
			run10 = 1;
		}
	}
	
	
}

void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		HAL_DMA_Abort_IT(&hdma_usart2_rx);
		if(strchr(rxBuffer,'F')!=NULL && strchr(rxBuffer,'A')!=NULL)
		{
			strcpy(positionData, rxBuffer);
			
		}
		memset(rxBuffer, 0, 256);
		HAL_UART_Receive_DMA(&huart2,rxBuffer,256);
		
	}
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

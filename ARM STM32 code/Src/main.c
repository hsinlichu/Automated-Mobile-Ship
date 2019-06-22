/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <ctype.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int flag = 1;
uint8_t rx_data = 'f';
uint8_t pulse_width = 0;
double front_distance = 40;
double last_front_distance = 40;
double left_distance = 40;
double last_left_distance = 40;
double right_distance = 40;
double last_right_distance = 40;
int front_rise = 1;
int right_rise = 1;
int left_rise = 1;
int isforward = 1;
int self_move = 1;
int speed = 10;
int left_side_threshold = 20;
int right_side_threshold = 20;
int turn_threshold = 70;
int delay_time = 900;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void initwifi(void);
void turn_right(int binf,int bin,int ainf,int ain);
void turn_left(int binf,int bin,int ainf,int ain);
void forward(int input);
void backward(int input);
void stop();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // set nsleep to high voltage
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3); // AIN right wheel
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); // BIN left wheel
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);                  // trigger
	if(HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_2) != HAL_OK){  // echo
		Error_Handler();
	}
	HAL_Delay(3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	if(HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2) != HAL_OK){
		Error_Handler();
	}
	HAL_Delay(3);
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
	if(HAL_TIM_IC_Start_IT(&htim15,TIM_CHANNEL_2) != HAL_OK){
		Error_Handler();
	}

	HAL_UART_Receive_IT(&huart4,&rx_data,1);
	HAL_UART_Receive_IT(&huart2,&rx_data,1);
	initwifi();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1){
		//uint8_t tmp[20];
		//sprintf(tmp,"%d %d\n\r",(int)left_distance,(int)last_left_distance);
		//HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
		if(flag){
			if(rx_data == 'm'){ // self_move
				self_move = 1;
				speed = 5;
				//char tmp[] = " self move\n\r";
				//HAL_UART_Transmit(&huart2,(uint8_t*)tmp,strlen(tmp),0xFFFF);
			}
			else if(rx_data == 'c'){ // user control
				self_move = 0;
				speed = 5;
				//char tmp[] = " user control\n\r";
				//HAL_UART_Transmit(&huart2,(uint8_t*)tmp,strlen(tmp),0xFFFF);
			}
			else if(rx_data == 'r'){ // turn right
				turn_right(speed,0,0,0); // origin turn_right(speed,0,speed-2,0);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				//stop();
			}
			else if(rx_data == 'l'){
				turn_left(0,0,speed,0); // origin turn_left(speed-2,0,speed,0);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				//stop();
			}
			else if(rx_data == 'b'){
				backward(speed);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				//stop();
			}
			else if(rx_data == 'x'){
				stop();
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
			else if(rx_data == 'f'){
				forward(speed);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				//stop();
			}
			else if(rx_data - '0' <= 5 && rx_data - '0' >= 0){ //(input >=0 && input <= 5){
				speed = rx_data - '0';
				forward(speed);
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
			flag = 0;
		}

		if(self_move){
			/*
			if(front_distance != 0 && front_distance < 15 && (front_distance - last_front_distance) < 0 && isforward){ // hit front // origin front_distance < 5
				uint8_t tmp[20];
				sprintf(tmp," back %d\n\r",(int)front_distance);
				HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

				do
					backward(speed);
				while(front_distance < turn_threshold || last_front_distance < turn_threshold ); // front_distance < 10
				sprintf(tmp,"out\n\r");
				HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

				forward(speed);
			}
			*/
			/*
			if(front_distance > turn_threshold && front_distance < 50 && isforward){ // hit front
				uint8_t tmp[20];
				sprintf(tmp,"reduce %d %lf\n\r",(int)front_distance,front_distance);
				HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				backward(speed);
				HAL_Delay(500);
				forward(speed - 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			}
			*/


			if(front_distance != 0 && last_front_distance != 0 && front_distance < turn_threshold && (front_distance - last_front_distance) < 0 && isforward){ // hit front
				uint8_t tmp[20];
				sprintf(tmp,"front turn %d %d\n\r",(int)front_distance,(int)last_front_distance);
				HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				if(right_distance < left_distance){
					sprintf(tmp,"turn left\n\r");
					HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
					while((front_distance < turn_threshold || last_front_distance < turn_threshold) || (right_distance - last_right_distance) < 0) //|| (right_distance - last_right_distance) < 0)
						turn_left(0,speed,speed,0);
				}
				else{
					sprintf(tmp,"turn right\n\r");
					HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
					while((front_distance < turn_threshold || last_front_distance < turn_threshold) || (left_distance - last_left_distance) < 0) // || (left_distance - last_left_distance) < 0)  && front_distance != last_front_distance
						turn_right(speed,0,0,speed);
				}
				sprintf(tmp,"out\n\r");
				HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				forward(speed);
			}

			if(right_distance != 0 && last_right_distance != 0 && (right_distance < right_side_threshold || last_right_distance < right_side_threshold ) && (right_distance - last_right_distance) < 0  && front_distance > turn_threshold && isforward){ // hit right
				uint8_t tmp[20];
				sprintf(tmp,"right stop %d %d\n\r",(int)right_distance,(int)last_right_distance);
				HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				do
					turn_left(0,speed,speed-4,0); //turn_left(0,speed,speed,0); // origin (right_distance < 5)?turn_left(speed-2,0,speed+1,0):turn_left(speed-2,0,speed,0);
				while((right_distance - last_right_distance) < 0 && (right_distance < right_side_threshold || last_right_distance < right_side_threshold ) && front_distance > turn_threshold);//&& front_distance != last_front_distance
				/*
				if(right_distance == last_right_distance)
					do
						turn_left(0,speed,speed,0);//backward(speed);
					while(front_distance < 15 && front_distance != last_front_distance);
*/
				//(front_distance < 100)?turn_left(0,speed,speed,0):turn_left(speed-2,0,speed,0); //
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				forward(speed);
			}
			if(left_distance != 0 && last_left_distance != 0 && (left_distance < left_side_threshold || last_left_distance < left_side_threshold ) && (left_distance - last_left_distance) < 0  && front_distance > turn_threshold && isforward){ // hit left
				uint8_t tmp[20];
				sprintf(tmp,"left stop %d %d\n\r",(int)left_distance,(int)last_left_distance);
				HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				do
					turn_right(speed,0,0,speed); //turn_right(speed,0,0,speed); // (left_distance < 5)?turn_right(speed+1,0,speed-2,0):turn_right(speed,0,speed-2,0);
				while((left_distance - last_left_distance) < 0 && (left_distance < left_side_threshold || last_left_distance < left_side_threshold ) && front_distance > turn_threshold);// && front_distance != last_front_distance
				/*

				if(left_distance - last_left_distance)
					do
						turn_right(speed,0,0,speed);//backward(speed);
					while(front_distance < 15 && front_distance != last_front_distance);
				*/
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
				forward(speed);
			}
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 17999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 719;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 719;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 10000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim15);

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart4){
		flag = 1;
		HAL_UART_Transmit(&huart2,&rx_data,1,0xFFFF);
		//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		HAL_UART_Receive_IT(&huart4,&rx_data,1);
	}
	if(huart == &huart2){
		HAL_UART_Transmit(&huart4,&rx_data,1,0xFFFF);
		//flag = 1;
		HAL_UART_Receive_IT(&huart2,&rx_data,1);
	}
}
void turn_right(int binf,int bin,int ainf,int ain){
	isforward = 1;
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	//uint8_t tmp[] = " Turn Right\n\r";
	//HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
	sConfigOC.Pulse = bin;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // BIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	sConfigOC.Pulse = binf;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

	sConfigOC.Pulse = ain;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // AIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	sConfigOC.Pulse = ainf; // 2
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}
void turn_left(int binf,int bin,int ainf,int ain){
	isforward = 1;
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	//uint8_t tmp[] = " Turn Left\n\r";
	//HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);

	sConfigOC.Pulse = bin;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // BIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	sConfigOC.Pulse = binf;  // 2
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

	sConfigOC.Pulse = ain;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // AIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	sConfigOC.Pulse = ainf;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}
void backward(int input){
	isforward = 0;
	//uint8_t tmp[] = " backward\n\r";
	//HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	sConfigOC.Pulse = input;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // BIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

	sConfigOC.Pulse = input - 2;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // AIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}
void stop(){
	isforward = 0;
	uint8_t tmp[] = " Stop\n\r";
	HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // BIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // AIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}
void forward(int input){
	isforward = 1;
	//uint8_t tmp[] = " forward\n\r";
	//HAL_UART_Transmit(&huart2,tmp,strlen((char*)tmp),0xFFFF);
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // BIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	sConfigOC.Pulse = input;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);

	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){ // AIN
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	sConfigOC.Pulse = input - 2; //right
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}

void initwifi(void){
	/*
	uint8_t tmp1[] = "AT\r\n";
	uint8_t tmp2[]= "AT+CWMODE=2\r\n";
	uint8_t tmp3[] = "AT+CWSAP=\"ESOE6\",";
	uint8_t tmp4[] = "\"123456789\",";
	uint8_t tmp5[] = "1,4\r\n";
	uint8_t tmp6[] = "AT+CIPMUX";
	uint8_t tmp7[] = "=1\r\n";
	uint8_t tmp8[] = "AT+CIPSE";
	uint8_t tmp9[] = "RVER=1,8080\r\n";
	uint8_t* cmd[] = {tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7,tmp8,tmp9};

	for(int i = 0;i < 9;i++){
		HAL_UART_Transmit(&huart4,cmd[i],strlen((char*)cmd[i]),0xFFFF);
		HAL_Delay(1000);
	}*/

	uint8_t tmmp[] = "AT\r\nAT+CWMODE=2\r\nAT+CWSAP=\"jameschu\",\"123456789\",1,4\r\nAT+CIPMUX=1\r\nAT+CIPSERVER=1,8080\r\n";
	for(int i = 0;i < strlen((char*)tmmp);++i){
		HAL_UART_Transmit(&huart4,&tmmp[i],1,0xFFFF);
		HAL_Delay(10);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim8){
		if(front_rise == 1){
			__HAL_TIM_SET_COUNTER(&htim8,0);
			front_rise = 0;
		}
		else{

			last_front_distance = front_distance;
			front_distance = __HAL_TIM_GET_COUNTER(&htim8);
			front_distance /= 5.8;

			front_rise = 1;
		}
	}
	else if(htim == &htim1){
		if(right_rise == 1){
			__HAL_TIM_SET_COUNTER(&htim1,0);
			right_rise = 0;
		}
		else{

			last_right_distance = right_distance;
			right_distance = __HAL_TIM_GET_COUNTER(&htim1);
			right_distance /= 5.8;
			right_rise = 1;
		}
	}
	else if(htim == &htim15){
		if(left_rise == 1){
			__HAL_TIM_SET_COUNTER(&htim15,0);
			left_rise = 0;
		}
		else{

			last_left_distance = left_distance;
			left_distance = __HAL_TIM_GET_COUNTER(&htim15);
			left_distance /= 5.8;
			left_rise = 1;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <time.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t value[8], adc_buffer[8], read_value[8];
uint16_t distribution[9];
uint16_t times = 2560;
uint8_t distribution_clear[9];
uint8_t distribution_output[9];
uint8_t distribution_output_pre[9];
uint8_t change[9];
uint8_t max_fall;
uint32_t t=0;
uint8_t screen[8];
//---------------------Interupt ADC for writing data and increase time register--------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{if (hadc->Instance==ADC1)
	{for(int i=0;i<8;i++)
		{value[i]=adc_buffer[i];
			t++;
		}
	}
}

//-----------------------------------------------------------------------------//
uint8_t disp1ay[38][8]={
{0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},//0
{0x10,0x30,0x50,0x10,0x10,0x10,0x10,0x7c},//1
{0x7E,0x2,0x2,0x7E,0x40,0x40,0x40,0x7E},//2
{0x3E,0x2,0x2,0x3E,0x2,0x2,0x3E,0x0},//3
{0x8,0x18,0x28,0x48,0xFE,0x8,0x8,0x8},//4
{0x3C,0x20,0x20,0x3C,0x4,0x4,0x3C,0x0},//5
{0x3C,0x20,0x20,0x3C,0x24,0x24,0x3C,0x0},//6
{0x3E,0x22,0x4,0x8,0x8,0x8,0x8,0x8},//7
{0x0,0x3E,0x22,0x22,0x3E,0x22,0x22,0x3E},//8
{0x3E,0x22,0x22,0x3E,0x2,0x2,0x2,0x3E},//9
{0x00,0xFC,0x22,0x21,0x21,0x22,0xFC,0x00},//A
{0x00,0x00,0xFE,0x91,0x91,0x91,0x6E,0x00},//B
{0x3C,0x40,0x40,0x40,0x40,0x40,0x40,0x3C},//C
{0x00,0x81,0xFF,0x81,0x81,0x42,0x3C,0x00},//D
{0x00,0x00,0xFF,0x91,0x91,0x91,0x91,0x00},//E
{0x7C,0x40,0x40,0x7C,0x40,0x40,0x40,0x40},//F
{0x3C,0x40,0x40,0x40,0x4c,0x44,0x44,0x3C},//G
{0x00,0xFF,0x08,0x08,0x08,0x08,0xFF,0x00},//H
{0x00,0x00,0x81,0x81,0xFF,0x81,0x81,0x00},//I
{0x3C,0x8,0x8,0x8,0x8,0x8,0x48,0x30},//J
{0x0,0x24,0x28,0x30,0x20,0x30,0x28,0x24},//K
{0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x7C},//L
{0x81,0xC3,0xA5,0x99,0x81,0x81,0x81,0x81},//M
{0x0,0x42,0x62,0x52,0x4A,0x46,0x42,0x0},//N
{0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},//O
{0x00,0xFF,0x11,0x11,0x11,0x0E,0x00,0x00},//P
{0x00,0x00,0xFF,0x11,0x11,0x11,0x0E,0x00},//Q (P_2)
{0x00,0xFF,0x11,0x11,0x19,0xE6,0x00,0x00},//R
{0x00,0x00,0x8E,0x91,0x91,0x91,0x61,0x00},//S
{0x00,0x00,0x01,0x01,0xFF,0x01,0x01,0x00},//T
{0x42,0x42,0x42,0x42,0x42,0x42,0x22,0x1C},//U
{0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18},//V
{0x00,0x49,0x49,0x49,0x49,0x2A,0x1C,0x00},//W
{0x41,0x22,0x14,0x08,0x14,0x22,0x41,0x00},//X
{0x00,0x01,0x02,0x04,0xF8,0x04,0x02,0x01},//Y
{0x0,0x7F,0x2,0x4,0x8,0x10,0x20,0x7F},//Z
};
//--------------------------Write byte to address----------------------------------------//
void write_byte(uint8_t byte)
{
	    for(int i=0;i<8;i++)
          {
	    	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);  // Pull the CLK LOW
	    	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, byte&0x80);// Write one BIT data MSB first
             byte = byte<<1;  // shift the data to the left
             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);  // Pull the CLK HIGH
           }
}
//----------------------------Write byte to a specific address-----------------------------------//
void write_max (uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);   // Pull the CS pin LOW
    write_byte(address);   //  write address
    write_byte(data);  //  write data
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);  // pull the CS HIGH
}
//-----------------------------Init for Matrix IC-------------------------------------//
void max_init(void)
{
 write_max(0x09, 0x00);       //  no decoding
 write_max(0x0a, 0x03);       //  brightness intensity
 write_max(0x0b, 0x07);       //  scan limit = 8 LEDs
 write_max(0x0c, 0x01);       //  power down =0,normal mode = 1
 write_max(0x0f, 0x00);       //  no test display
}
//------------------------------Write a string on matrix display--------------------------------//
void write_string (char *str)
{
	while (*str)
	{
		for(int i=1;i<9;i++)
			   {
			       write_max (i,disp1ay[(*str - 55)][i-1]);
			   }
		*str++;
		HAL_Delay (500);
	}
}
//----------Display to the screen using data in array screen[]---------
void write_screen (void)
{
		for(int i=1;i<9;i++)
			   {
			       write_max (i,screen[i-1]);
			   }
}
//---------Function used to calculate power of 2------------
int power_2(int value)
{ int output=1;
	for (int i=0;i<value;i++)
	{ output*=2;
	}
	return(output);
}
//------------Find the value of byte need to be written with the value of x------------------
int column(int x)
{return((power_2(x)-1)*power_2(8-x));
}
//-------------Calculate the display value of each column based on distribution value--------------
void display_2 (uint8_t distribution[9])
	{for(int i=0;i<9;i++)
		screen[i]=column(distribution[i]);
	}
//------------Save for test, used for display scanning column to column---------------
void falling (int col,int destination, int time)
	{display_2(distribution_output_pre);
	uint8_t drop =1;
	uint8_t temp=0;
	for (int i=8;i>destination-1;i--)
	{HAL_Delay(time);
	temp=screen[col-1]+drop;
	write_max(col,temp);
	drop*=2;
	}
}
//-----------Find the minumum distribution for calculate the max drop steps-----------------
uint8_t max(uint8_t distribution_output[9],uint8_t distribution_output_pre[9])
{ uint8_t k=8;
	for (int i=0;i<9;i++)
	{if (distribution_output[i]>distribution_output_pre[i])
			{if (distribution_output[i]<k)
				{k=distribution_output[i];
				}
			}
	}
	return(9-k);
}
//----------Check each column for next scanning time changes----------------------
void check_change(uint8_t distribution_output[9],uint8_t distribution_output_pre[9])
{	for (int i=0;i<9;i++)
	{if (distribution_output[i]>distribution_output_pre[i])
			{ change[i]=1;
				goto out;
			}
		change[i]=0;
		out:continue;
			}
}
//----------Graphic falling code----------------------------------
void falling_new (uint8_t k, int time)
{ display_2(distribution_output_pre);
	uint8_t step=1;
	int time_reduce_step[7] = {1,1,3,0,1,1,0};
	int time_reduce=0;
	int iteration=0;
	for(int j=k;j>0;j--)
	{
	uint8_t tmp[9];
	for (int i=0;i<9;i++)
	tmp[i]=0;
	
	for(int i=0;i<9;i++)
	{ tmp[i]=screen[i];
		if (change[i]==1)
		{ if(j>(k+distribution_output[i])-8)
			{tmp[i]=screen[i]+step;
				goto next_2;}
			tmp[i]=screen[i]+power_2(8-distribution_output[i]);
				next_2:continue;
		}
	}
	for (int i=0;i<9;i++)
	write_max(i+1,tmp[i]);
	step*=2;
	HAL_Delay(time-time_reduce);
	time_reduce+=time_reduce_step[iteration];
	iteration++;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

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
	MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA (&hadc1, (uint32_t*)adc_buffer,8);
	max_init ();
	for (int i=0;i<9;i++)
	{distribution_clear[i]=0;
	}
  /* USER CODE END 2 */
  write_string ("HAPQY");//Q=P2
	for (int i=0;i<9;i++)
	write_max(i+1,0);
	HAL_Delay(500);
	write_string ("BIRTHDAY");
	HAL_Delay(500);
	write_string ("SISTER");
	HAL_Delay(500);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {display_2(distribution_clear);
	write_screen();
	for (int i = 0; i < 9; i++)
    distribution[i] = 0;
  /* USER CODE END WHILE */
	for (int i=0;i<times;i++)
		{ uint16_t value_temp[8];
			for (int j=0;j<8;j++)
			value_temp[j]=value[j];
			uint8_t temp=0;
			for (int j=0;j<8;j++)
			{ 
				srand(t+value_temp[j]);
				temp+=(((uint16_t)rand()%200)/100);
			}
			if (t>(2000000))
				{t=rand()/1000000;
				}
			distribution[temp]++;
			//-------------Scanning column falling---------------
			/*if(i%100==0)
			{ 
				for (int i=0; i<9;i++)
				{ distribution_output[i]=(float)distribution[i]*1072.0/35.0/(float)times+0.5;
					if (distribution_output[i]>distribution_output_pre[i])
					{falling(i+1,distribution_output[i],20);
					}
				}
				
				for(int i=0;i<9;i++)
				{distribution_output_pre[i]=distribution_output[i];
				}
			}*/
			//-------------Synchronous falling---------------
			if(i%100==0)
			{	uint8_t k;
				for (int i=0; i<9;i++)
				 {distribution_output[i]=(float)distribution[i]*1072.0/35.0/(float)times+0.5;
					if (distribution_output[i]>8)
					{distribution_output[i]=8;
					}
				 }
			check_change(distribution_output,distribution_output_pre);
			k=max(distribution_output,distribution_output_pre);
			falling_new(k,26);
				 
			for(int i=0;i<9;i++)
				{distribution_output_pre[i]=distribution_output[i];
				}
			}				
		}
		//--------------End of graphic display--------------
		for (int i=0; i<9;i++)
		{ distribution_output[i]=(float)distribution[i]*1072.0/35.0/(float)times+0.5;
		}
    /* USER CODE BEGIN 3 */
		//--------------Keep the screen and wait for next display-------------
		display_2(distribution_output);
		write_screen();
		falling(1, 0, 400);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, clock_Pin|cs_Pin|data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : clock_Pin cs_Pin data_Pin */
  GPIO_InitStruct.Pin = clock_Pin|cs_Pin|data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

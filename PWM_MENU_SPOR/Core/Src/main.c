/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float deger;
char line1[32];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint16_t parlaklik1;
uint16_t parlaklik2;
uint16_t parlaklik3;
uint16_t sayac1,sayac2,sayac3,sayac4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init(_LCD_4BIT, _LCD_FONT_5x8, _LCD_2LINE);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,parlaklik1);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,parlaklik2);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,parlaklik3);
	  //AYARLAR
	  HAL_GPIO_WritePin(GPIOA, MENULED_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(GPIOA, MENULED_Pin, GPIO_PIN_RESET);
	  lcd_clear();
	  lcd_print(1,1,"      MENU");
	  HAL_Delay(500);
	  if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
	  {
		          HAL_Delay(200);
		          sayac1=1;


	  }

	           while(sayac1==1)
	           {

		          lcd_clear();
		  		  lcd_print(1,1, "    AYARLAR");
		  		  lcd_print(2,1, " LED1 LED2 LED3 ");
                  HAL_Delay(500);
                  if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
                  	  {
                  		          HAL_Delay(200);
                  		          sayac1=0;
                  	  }

		  		if(HAL_GPIO_ReadPin(GPIOB, buton3_Pin))
		  			  {
		  				     HAL_Delay(200);
		  				     sayac2++;
		  			  }

		  		if(HAL_GPIO_ReadPin(GPIOB, buton1_Pin))
		  			  {
		  				  	 HAL_Delay(200);
		  				  	 sayac2--;
		  			  }


		  		 while(sayac2 == 1)
		  		 {
		  					  		if(HAL_GPIO_ReadPin(GPIOB, buton3_Pin))
		  					  			  {
		  					  				     HAL_Delay(200);
		  					  				     sayac2++;
		  					  			  }

		  					  		if(HAL_GPIO_ReadPin(GPIOB, buton1_Pin))
		  					  			  {
		  					  				  	 HAL_Delay(200);
		  					  				  	 sayac2--;
		  					  			  }

		  			  HAL_Delay(200);
		  			  lcd_clear();
		  			  lcd_print(1,1, "    AYARLAR");
		  			  lcd_print(2,1, "     >LED1< ");
		  			  HAL_Delay(500);
		  			  if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
		  			  {
		  				HAL_Delay(250);
		  				sayac3=1 ;
		  			  }

		  			  while(sayac3 ==1 )
		  			  {
		  				if(HAL_GPIO_ReadPin(GPIOB, buton3_Pin))
		  						  			  {
                         HAL_Delay(200);
                          parlaklik1++;
                          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,parlaklik1);
		  						  			  }

		  						  		if(HAL_GPIO_ReadPin(GPIOB, buton1_Pin))
		  						  				  			  {
		  						  				  				HAL_Delay(200);
		  						  				  				parlaklik1--;
		  						  				  			    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,parlaklik1);
		  						  				  			  }
		  						  	if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
		  						  			  			  {
		  						  			  				HAL_Delay(250);
		  						  			  				sayac3=0;
		  						  			  				sayac2=0;
		  						  			  			  }

		  						  		  					  sprintf(line1,"SET= %d",parlaklik1);
		  						  		  					  lcd_clear();
		  						  		  					  lcd_print(1,1,line1);
		  						  		  					  lcd_print(2,1,"     P W M");
                                                              HAL_Delay(500);
		  			  }
		  		    }
		  		while(sayac2 == 2)
		  				  		 {
		  					  		if(HAL_GPIO_ReadPin(GPIOB, buton3_Pin))
		  					  			  {
		  					  				     HAL_Delay(200);
		  					  				     sayac2++;
		  					  			  }

		  					  		if(HAL_GPIO_ReadPin(GPIOB, buton1_Pin))
		  					  			  {
		  					  				  	 HAL_Delay(200);
		  					  				  	 sayac2--;
		  					  			  }

		  			                  HAL_Delay(200);
		  				  			  lcd_clear();
		  				  			  lcd_print(1,1, "    AYARLAR");
		  				  			  lcd_print(2,1, "     >LED2< ");
		  				  			  HAL_Delay(500);
		  				  			  if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
		  				  			  {
		  				  				HAL_Delay(250);
		  				  				sayac3=1 ;
		  				  			  }

		  				  			  while(sayac3 ==1 )
		  				  			  {
		  				  				if(HAL_GPIO_ReadPin(GPIOB, buton3_Pin))
		  				  						  			  {
		  		                         HAL_Delay(200);
		  		                          parlaklik2++;
		  		                          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,parlaklik2);
		  				  						  			  }

		  				  						  		if(HAL_GPIO_ReadPin(GPIOB, buton1_Pin))
		  				  						  				  			  {
		  				  						  				  				HAL_Delay(200);
		  				  						  				  				parlaklik2--;
		  				  						  				  			    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,parlaklik2);
		  				  						  				  			  }
		  				  						  	if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
		  				  						  			  			  {
		  				  						  			  				HAL_Delay(250);
		  				  						  			  				sayac3=0;
		  				  						  			  				sayac2=0;
		  				  						  			  			  }

		  				  						  		  					  sprintf(line1,"SET= %d",parlaklik2);
		  				  						  		  					  lcd_clear();
		  				  						  		  					  lcd_print(1,1,line1);
		  				  						  		  					  lcd_print(2,1,"     P W M");
		  		                                                              HAL_Delay(500);
		  				  			  }
		  				  		 }
		  		while(sayac2 == 3)
		  				  				  		 {
		  					  		if(HAL_GPIO_ReadPin(GPIOB, buton3_Pin))
		  					  			  {
		  					  				     HAL_Delay(200);
		  					  				     sayac2++;
		  					  			  }

		  					  		if(HAL_GPIO_ReadPin(GPIOB, buton1_Pin))
		  					  			  {
		  					  				  	 HAL_Delay(200);
		  					  				  	 sayac2--;
		  					  			  }

		  			                                  HAL_Delay(200);
		  				  				  			  lcd_clear();
		  				  				  			  lcd_print(1,1, "    AYARLAR");
		  				  				  			  lcd_print(2,1, "     >LED3< ");
		  				  				  			  HAL_Delay(500);
		  				  				  			  if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
		  				  				  			  {
		  				  				  				HAL_Delay(250);
		  				  				  				sayac3=1 ;
		  				  				  			  }

		  				  				  			  while(sayac3 ==1 )
		  				  				  			  {
		  				  				  				if(HAL_GPIO_ReadPin(GPIOB, buton3_Pin))
		  				  				  						  			  {
		  				  		                         HAL_Delay(200);
		  				  		                          parlaklik3++;
		  				  		                          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,parlaklik3);
		  				  				  						  			  }

		  				  				  						  		if(HAL_GPIO_ReadPin(GPIOB, buton1_Pin))
		  				  				  						  				  			  {
		  				  				  						  				  				HAL_Delay(200);
		  				  				  						  				  				parlaklik3--;
		  				  				  						  				  			    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,parlaklik3);
		  				  				  						  				  			  }
		  				  				  						  	if(HAL_GPIO_ReadPin(GPIOB, buton2_Pin))
		  				  				  						  			  			  {
		  				  				  						  			  				HAL_Delay(250);
		  				  				  						  			  				sayac3=0;
		  				  				  						  			  				sayac2=0;
		  				  				  						  			  			  }
		  				  				  						  	                          sayac2=0;
		  				  				  						  		  					  sprintf(line1,"SET= %d",parlaklik3);
		  				  				  						  		  					  lcd_clear();
		  				  				  						  		  					  lcd_print(1,1,line1);
		  				  				  						  		  					  lcd_print(2,1,"     P W M");
		  				  		                                                              HAL_Delay(500);
		  				  				  			  }
		  				  				  		 }

	              }




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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin|MENULED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RS_Pin LCD_D4_Pin LCD_D5_Pin
                           LCD_D6_Pin LCD_D7_Pin MENULED_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin|MENULED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : buton1_Pin buton2_Pin buton3_Pin */
  GPIO_InitStruct.Pin = buton1_Pin|buton2_Pin|buton3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

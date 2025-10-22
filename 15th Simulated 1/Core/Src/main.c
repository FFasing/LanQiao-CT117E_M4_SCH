/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
u8 lcd_mod=0,direction=0;
double V_R37,distance=80.3;
u8 B1=0,B2=0,B3=0,B4=0,B11=0,B22=0,B33=0,B44=0;
u16 TIM_5S=0,TIM_5S_CON=0,TIM_100=0;
u8 L_OVER_FLAG=0,R_OVER_FLAG=0;
u8 LD1=0,LD2=0;
u8 rx[10],rx_len=0,rx_count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM17)
	{
		if(TIM_5S_CON==1&&lcd_mod==0)
		if(++TIM_5S==5000)
		{
			char text[20];
			sprintf(text,"Warn\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)text,6,HAL_MAX_DELAY);
			direction=0;
			lcd_mod=1;
			TIM_5S=LD1=LD2=TIM_100=0;
			TIM_5S_CON=0;
		}
		
		if(direction==0&&lcd_mod==0)
		{
			TIM_5S=LD1=LD2=TIM_100=0;
		}
		else if(direction==2&&lcd_mod==0)
		{
			if(++TIM_100==100)
			{
				LD2=0;
				LD1^=1;
				TIM_100=0;
			}
		}
		else if(direction==1&&lcd_mod==0)
		{
			if(++TIM_100==100)
			{
				LD1=0;
				LD2^=1;
				TIM_100=0;
			}
		}
	}
}
void lcd_proc(void)
{
	char text[21];
	if(lcd_mod==0)
	{
		sprintf(text,"        DATA        ");
		LCD_DisplayStringLine(Line1, (u8 *)text);
		if(direction==0) sprintf(text,"       N:S        ");
		else if(direction==1) sprintf(text,"       N:R        ");
		else if(direction==2) sprintf(text,"       N:L        ");
		LCD_DisplayStringLine(Line3, (u8 *)text);
		sprintf(text,"       D:%.1f        ",distance);
		LCD_DisplayStringLine(Line4, (u8 *)text);
	}
	if(lcd_mod==1)
	{
		sprintf(text,"                    ");
		LCD_DisplayStringLine(Line1, (u8 *)text);
		sprintf(text,"                    ");
		LCD_DisplayStringLine(Line3, (u8 *)text);
		sprintf(text,"        WARN        ");
		LCD_DisplayStringLine(Line4, (u8 *)text);
	}
}
void key_proc(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0) B1=1;
	else B1=0;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0) B2=1;
	else B2=0;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0) B3=1;
	else B3=0;
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0) B4=1;
	else B4=0;
	if(B1==1&&B11==0)
	{
		B11=1;
		if(lcd_mod==1) 
		{
			LCD_Clear(Black);
			char text[20];
			sprintf(text,"Success\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)text,9,HAL_MAX_DELAY);
			LCD_Clear(Black);
			lcd_mod=0;
		}
	}
	if(B11==1&&B1==0) B11=0;
	
	if(B3==1&&B33==0)
	{
		B33=1;
		if(lcd_mod==0)
		{
			if(direction==0) 
			{
				char text[20];
				sprintf(text,"Warn\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)text,6,HAL_MAX_DELAY);
				LCD_Clear(Black);
				lcd_mod=1;
			}
			else if(direction==2&&TIM_5S_CON==1)
			{
				char text[20];
				sprintf(text,"Success\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)text,9,HAL_MAX_DELAY);
				TIM_5S_CON=0;
				L_OVER_FLAG=1;
				direction=0;
			}
		}
		
	}
	if(B33==1&&B3==0) B33=0;
	
	if(B4==1&&B44==0)
	{
		B44=1;
		if(lcd_mod==0)
		{
			if(direction==0) 
			{
				char text[20];
				sprintf(text,"Warn\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)text,6,HAL_MAX_DELAY);
				LCD_Clear(Black);
				lcd_mod=1;
			}
			else if(direction==1&&TIM_5S_CON==1)
			{
				char text[20];
				sprintf(text,"Success\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)text,9,HAL_MAX_DELAY);
				TIM_5S_CON=0;
				R_OVER_FLAG=1;
				direction=0;
			}
		}
	}
	if(B44==1&&B4==0) B44=0;
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
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim17);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);
	 LCD_Clear(Black);
	  HAL_GPIO_WritePin(GPIOC,(uint16_t)0xFF00,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
  while (1)
  {
	  HAL_ADC_Start(&hadc2);
	  V_R37=HAL_ADC_GetValue(&hadc2)/4095.0*3.3;
	  if(V_R37<=3.0)	distance=V_R37*100.0;
	  else distance=300;
	  lcd_proc();
	  key_proc();
	  HAL_GPIO_WritePin(GPIOC,(uint16_t)0xff00,1);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,!LD1);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,!LD2);
	  if(lcd_mod==1)  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
	  else  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
	  if(TIM_5S_CON==1)	memset(rx,'\0',sizeof(rx));
	  if(TIM_5S_CON==0&&lcd_mod==0&&strcmp((char *)rx,"")!=0)
	  {
		  if(strcmp((char *)rx,"L")==0)
		  {
			  direction=2;
			  memset(rx,'\0',sizeof(rx));
			  TIM_5S_CON=1;
		  }
		  else if(strcmp((char *)rx,"R")==0)
		  {
			  direction=1;
			  memset(rx,'\0',sizeof(rx));
			  TIM_5S_CON=1;
		  }
		  else 
		  {
			   memset(rx,'\0',sizeof(rx));
				char text[20];
				sprintf(text,"ERROR\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)text,7,HAL_MAX_DELAY);
		  }
	  }
	  if(TIM_5S_CON==0&&lcd_mod==1&&strcmp((char *)rx,"")!=0)
	  {
			memset(rx,'\0',sizeof(rx));
			char text[20];
			sprintf(text,"WAIT\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)text,6,HAL_MAX_DELAY);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

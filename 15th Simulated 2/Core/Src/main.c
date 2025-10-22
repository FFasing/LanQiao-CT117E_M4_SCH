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
u8 lcd_mod=0;
double 	temp=23.5,V_R37=0.0;
u8 mode=0,gear=1;
int TIM_5S,TIM_5S_CON,TIM_5S_FLAG;
int TIM_3S,TIM_3S_CON,UART_TRANSMIT_FLAG;
u8 B1,B11,B2,B22,B3,B33,B4,B44;
u8 rx[10];
u8 rx_len=0;
u8 rx_count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart_proc(void)
{
	if(lcd_mod==0&&strcmp((char *)rx,"")!=0)
	{
			if(strcmp((char *)rx,"B1")==0) 
			{
				UART_TRANSMIT_FLAG=1;
				TIM_5S_CON=0;
				TIM_5S=0;
				mode^=1;
				memset(rx,'\0',sizeof(rx));
			}
			else if(strcmp((char *)rx,"B2")==0) 
			{
				UART_TRANSMIT_FLAG=1;
				TIM_5S_CON=0;
				TIM_5S=0;
				gear++;
				if(gear>=3) gear=3;
				memset(rx,'\0',sizeof(rx));
			}
			else if(strcmp((char *)rx,"B3")==0)
			{
				UART_TRANSMIT_FLAG=1;
				TIM_5S_CON=0;
				TIM_5S=0;
				gear--;
				if(gear<=1) gear=1;
				memset(rx,'\0',sizeof(rx));
			}
			else
			{
				HAL_UART_Transmit(&huart1,"NULL",4,HAL_MAX_DELAY);
				memset(rx,'\0',sizeof(rx));
			}
	}
	if(lcd_mod==1&&strcmp((char *)rx,"")!=0)
	{
			if(strcmp((char *)rx,"B1")==0||strcmp((char *)rx,"B2")==0||strcmp((char *)rx,"B3")==0)
			{
				UART_TRANSMIT_FLAG=1;
				TIM_5S_CON=0;
				TIM_5S=0;
				LCD_Clear(Black);
				HAL_Delay(10);
				lcd_mod=0;
				memset(rx,'\0',sizeof(rx));
			}
			else
			{
				HAL_UART_Transmit(&huart1,"NULL",4,HAL_MAX_DELAY);
				memset(rx,'\0',sizeof(rx));
			}
	}
}
void lcd_proc(void)
{
	char text[20];
	if(lcd_mod==0)
	{
		sprintf(text,"        DATA      ");
		LCD_DisplayStringLine(Line1, (unsigned char *)text);
		sprintf(text,"     TEMP:%.1f    ",temp);
		LCD_DisplayStringLine(Line3, (unsigned char *)text);
		if(mode==0)	sprintf(text,"     MODE:Auto     ");
		if(mode==1)	sprintf(text,"     MODE:Manu     ");
		LCD_DisplayStringLine(Line4, (unsigned char *)text);
		if(mode==0&&temp<25&&mode==0)	gear=1;
		if(mode==0&&temp>=25&&temp<=30&&mode==0)	gear=2;
		if(mode==0&&temp>30&&mode==0)	gear=3;
		sprintf(text,"     GEAR:%d    ",gear);
		LCD_DisplayStringLine(Line5, (unsigned char *)text);
	}
	else
	{
		sprintf(text,"     SLEEPING      ");
		LCD_DisplayStringLine(Line4, (unsigned char *)text);
		sprintf(text,"     TEMP:%.1f    ",temp);
		LCD_DisplayStringLine(Line5, (unsigned char *)text);
		
	}
}
void key_proc(void)
{
	u8 key=0;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0)	B1=1;
	else B1=0;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)	B2=1;
	else B2=0;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0)	B3=1;
	else B3=0;
	if(B1==1&&B11==0)
	{B11=1;
		if(lcd_mod==1)	key=1;
		else
		{
			key=1;
			mode^=1;
		}
	}if(B11==1&&B1==0)	B11=0;
	
	if(B2==1&&B22==0)
	{B22=1;
		if(lcd_mod==1)	key=1;
		else
		{
			key=1;
			if(mode==1)
			{
				gear++;
				if(gear>=3) gear=3;
			}
		}
	}if(B22==1&&B2==0)	B22=0;
	
	if(B3==1&&B33==0)
	{B33=1;
		if(lcd_mod==1)	key=1;
		else
		{
			key=1;
			if(mode==1)
			{
				gear--;
				if(gear<=1) gear=1;
			}
		}
	}if(B33==1&&B3==0)	B33=0;
	

	if(key==1&&lcd_mod==0)	
	{
		TIM_5S_CON=0;
		TIM_5S=0;
	}
	else if(key==0&&lcd_mod==0)	TIM_5S_CON=1;
	
	if(key==0&&lcd_mod==1)
	{
		TIM_5S_CON=0;
		TIM_5S=0;
	}
	else if(key==1&&lcd_mod==1)
	{
		LCD_Clear(Black);
		lcd_mod=0;
		TIM_5S_CON=0;
		TIM_5S=0;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM17)
	{
		if(TIM_5S_CON==1)
		if(++TIM_5S==5000)
		{
			TIM_5S=0;
			TIM_5S_FLAG=1;
		}
		if(TIM_3S_CON==1)
		if(++TIM_3S==3000)
		{
			TIM_3S=0;
			TIM_3S_CON=0;
		}
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
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	HAL_TIM_Base_Start_IT(&htim17);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	TIM2->PSC=100-1;
	TIM2->ARR=400-1;
  while (1)
  {
	  uart_proc();
	  HAL_ADC_Start(&hadc2);
	  V_R37=HAL_ADC_GetValue(&hadc2)/4095.0*3.3;
	  temp=10.0*V_R37+10.0;
	  if(V_R37<=1) temp=20;
	  if(V_R37>=3) temp=40;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(TIM_5S_FLAG==1)
	  {
		  TIM_5S_FLAG=0;
		  LCD_Clear(Black);
		  lcd_mod=1;
	  }
	  lcd_proc();key_proc();
	  uint32_t pulse;
	  if(gear==1)	pulse = (htim2.Instance->ARR + 1) * 0.1;
	  if(gear==2)	pulse = (htim2.Instance->ARR + 1) * 0.4;
	  if(gear==3)	pulse = (htim2.Instance->ARR + 1) * 0.8;
	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,pulse);
	  
	  HAL_GPIO_WritePin(GPIOC,(uint16_t)0xff00,1);
	  if(mode==0)	HAL_GPIO_WritePin(GPIOC,(uint16_t)0x8000,0);
	  if(gear==1)	HAL_GPIO_WritePin(GPIOC,(uint16_t)0x0100,0);
	  if(gear==2)	HAL_GPIO_WritePin(GPIOC,(uint16_t)0x0200,0);
	  if(gear==3)	HAL_GPIO_WritePin(GPIOC,(uint16_t)0x0400,0);
	  if(UART_TRANSMIT_FLAG==1)	
	  {
		  UART_TRANSMIT_FLAG=0;
		  TIM_3S_CON=1;
	  }
	  HAL_GPIO_WritePin(GPIOC,(uint16_t)0x0800,!TIM_3S_CON);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0);
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

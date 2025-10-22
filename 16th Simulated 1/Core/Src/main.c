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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
u8 lcd_mod=0;
u8 pass_word[3]={'*','*','*'};
int pass_word_int[3]={0,0,0};
u8 PASS_WORD_INDEX=0;
u8 b1,b11;
double V_R37;
int TIM_3S=0,TIM_3S_CON=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM17)
	{
		if(TIM_3S_CON==1)
			if(++TIM_3S==3000)
			{
				TIM_3S_CON=0;
				LCD_Clear(Black);
				PASS_WORD_INDEX=0;
				int i;for(i=0;i<3;i++) pass_word[i]='*';
				lcd_mod=0;
			}
	}
}

void write_24c02(uint8_t addr,uint8_t data)
{
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	I2CSendByte(addr);
	I2CWaitAck();
	I2CSendByte(data);
	I2CWaitAck();
	I2CStop();
	HAL_Delay(50);
}

uint8_t read_24c02(uint8_t addr)
{
	uint8_t data;
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	I2CSendByte(addr);
	I2CWaitAck();
	I2CStop();
	HAL_Delay(50);
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	data = I2CReceiveByte();
	I2CSendNotAck();
	I2CStop();
	HAL_Delay(50);
	return data;
}
void lcd_proc(void)
{
	u8 text[20];
	if(lcd_mod==0)
	{
		TIM_3S_CON=0;
		sprintf((char *)text,"        Lock       ");
		LCD_DisplayStringLine(Line1, (unsigned char *)text);
		sprintf((char *)text,"      Pass Word    ");
		LCD_DisplayStringLine(Line3, (unsigned char *)text);
		sprintf((char *)text,"        %c %c %c      ",pass_word[0],pass_word[1],pass_word[2]);
		LCD_DisplayStringLine(Line4, (unsigned char *)text);
	}
	if(lcd_mod==1)
	{
		TIM_3S_CON=1;
		sprintf((char *)text,"        Set       ");
		LCD_DisplayStringLine(Line1, (unsigned char *)text);
		sprintf((char *)text,"       Change     ");
		LCD_DisplayStringLine(Line3, (unsigned char *)text);
		sprintf((char *)text,"        %c %c %c      ",pass_word[0],pass_word[1],pass_word[2]);
		LCD_DisplayStringLine(Line4, (unsigned char *)text);
	}
//		sprintf((char *)text,"        %c %c %c      ",pass_word_int[0]+48,pass_word_int[1]+48,pass_word_int[2]+48);
//		LCD_DisplayStringLine(Line6, (unsigned char *)text);
}
void key_proc(void)
{
	//---------------------------KEY---------------------------//
	int i,y=0;
	if(HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)==0) b1=1;
	else b1=0;
	if(b1==1&&b11==0)
	{
		b11=1;
		PASS_WORD_INDEX++;
		TIM_3S=0;
		if(lcd_mod==0)
		{
			if(PASS_WORD_INDEX==3)
			{
				PASS_WORD_INDEX=0;
				LCD_Clear(Black);
				y=1;
				for(i=0;i<3;i++)
				{
					if(pass_word[i]!=pass_word_int[i]+48) y=0;
				}
				if(y==1)
				{
					LCD_Clear(Black);
					lcd_mod=1;
				}
					for(i=0;i<3;i++) pass_word[i]='*';
			}
		}
		if(lcd_mod==1)
		{
			if(PASS_WORD_INDEX!=0)
			{
				pass_word_int[PASS_WORD_INDEX-1]=pass_word[PASS_WORD_INDEX-1]-48;
				if(PASS_WORD_INDEX-1==0) write_24c02(0,pass_word_int[PASS_WORD_INDEX-1]);
				if(PASS_WORD_INDEX-1==1) write_24c02(1,pass_word_int[PASS_WORD_INDEX-1]);
				if(PASS_WORD_INDEX-1==2) write_24c02(2,pass_word_int[PASS_WORD_INDEX-1]);
				TIM_3S=0;
			}
			if(PASS_WORD_INDEX==3)
			{
				PASS_WORD_INDEX=0;
				for(i=0;i<3;i++) pass_word[i]='*';
				LCD_Clear(Black);
				lcd_mod=0;
			}
		}
	}if(b1==0&&b11==1) b11=0;
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
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	HAL_TIM_Base_Start_IT(&htim17);
	I2CInit();
	HAL_Delay(100);
	write_24c02(0,0);
	write_24c02(1,1);
	write_24c02(2,2);
	HAL_Delay(100);
	pass_word_int[0]=read_24c02(0);
	pass_word_int[1]=read_24c02(1);
	pass_word_int[2]=read_24c02(2);
	while(1)
	{
	//---------------------------ADC---------------------------//
	HAL_ADC_Start(&hadc2);
	V_R37=HAL_ADC_GetValue(&hadc2)/4095.0*3.3;
	HAL_ADC_Stop(&hadc2);
	if(V_R37<1.5) pass_word[PASS_WORD_INDEX]=0+48;
	if(V_R37>=1.5&&V_R37<=2.5) pass_word[PASS_WORD_INDEX]=1+48;
	if(V_R37>2.5) pass_word[PASS_WORD_INDEX]=2+48;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  lcd_proc();
	  key_proc();
		
	HAL_GPIO_WritePin(GPIOC,(uint16_t)0xff00,GPIO_PIN_SET);
	if(lcd_mod==0) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
	if(lcd_mod==1) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
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

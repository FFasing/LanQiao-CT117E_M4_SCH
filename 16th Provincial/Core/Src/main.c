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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void e2prom_write(u8 addr,u8 data)
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
u8 e2prom_read(u8 addr)
{
	u8 data;
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	I2CSendByte(addr);
	I2CWaitAck();
	HAL_Delay(50);
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	data=I2CReceiveByte();
	I2CWaitAck();
	I2CStop();
	HAL_Delay(50);
	return data;
}
u8 lcd_mod=0;
u8 key_read;
u8 b1,b11,b2,b22,b3,b33,b4,b44;
double V_R37,V_R38;
double F_R39,F_R40;
u8 e2p_data[5]={'0','0','0'},e2p_index=0;
int tim500,tim_b3_con,tim_b4_con,tim_ok;
void key(void)
{	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0) b1=1;
	else b1=0;
	if(b1==1&&b11==0)
	{
		b11=1;key_read=1;
		lcd_mod++;
		LCD_Clear(Black);
		if(lcd_mod==2) lcd_mod=0;
	}if(b1==0&&b11==1) b11=0;
	
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0) b2=1;
	else b2=0;
	if(b2==1&&b22==0)
	{
		b22=1;key_read=2;
		e2p_index++;
		if(e2p_index==3) e2p_index=0;
	}if(b2==0&&b22==1) b22=0;
	
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0) b3=1;
	else b3=0;
	if(b3==1&&b33==0)
	{
		b33=1;key_read=3;
		tim_b3_con=1;
		tim_ok=0;
	}if(b3==0&&b33==1) 
	{
		if(tim_ok==0) e2p_data[e2p_index]++;
		tim_b3_con=0;
		tim500=0;
		e2prom_write(e2p_index,e2p_data[e2p_index]);
		b33=0;
	}
	
	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0) b4=1;
	else b4=0;
	if(b4==1&&b44==0)
	{
		b44=1;key_read=4;
		e2prom_write(e2p_index,e2p_data[e2p_index]);
		tim_b4_con=1;
		tim_ok=0;
	}if(b4==0&&b44==1)
	{
		if(tim_ok==0) e2p_data[e2p_index]--;
		tim_b4_con=0;
		tim500=0;
		e2prom_write(e2p_index,e2p_data[e2p_index]);
		b44=0;
	}
}
void lcd(void)
{
	u8 t[30];
	if(lcd_mod==0)
	{
		sprintf((char *)t,"        BASE        ");
		LCD_DisplayStringLine(Line1, (unsigned char *)t);
		if(key_read==0) sprintf((char *)t," KEY:         ");
		if(key_read==1) sprintf((char *)t," KEY:B1       ");
		if(key_read==2) sprintf((char *)t," KEY:B2       ");
		if(key_read==3) sprintf((char *)t," KEY:B3       ");
		if(key_read==4) sprintf((char *)t," KEY:B4       ");
		LCD_DisplayStringLine(Line3, (unsigned char *)t);
		sprintf((char *)t," V_R37:%.1f V_R38:%.1f  ",V_R37,V_R38);
		LCD_DisplayStringLine(Line4, (unsigned char *)t);
		if(F_R39<1000)		sprintf((char *)t," F_R39:%.1fHz  ",F_R39);
		if(F_R39>=1000)		sprintf((char *)t," F_R39:%.1fKHz ",F_R39/1000.0);
		LCD_DisplayStringLine(Line5, (unsigned char *)t);
		if(F_R40<1000)		sprintf((char *)t," F_R40:%.1fHz  ",F_R40);
		if(F_R40>=1000)		sprintf((char *)t," F_R40:%.1fKHz ",F_R40/1000.0);
		LCD_DisplayStringLine(Line6, (unsigned char *)t);
		if(e2p_index==0)	sprintf((char *)t," E2P:[%c]  %c   %c   ",e2p_data[0],e2p_data[1],e2p_data[2]);
		if(e2p_index==1)	sprintf((char *)t," E2P: %c  [%c]  %c   ",e2p_data[0],e2p_data[1],e2p_data[2]);
		if(e2p_index==2)	sprintf((char *)t," E2P: %c   %c  [%c]  ",e2p_data[0],e2p_data[1],e2p_data[2]);
		LCD_DisplayStringLine(Line7, (unsigned char *)t);
		
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		F_R40=1000000.0/(HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1)+1.0);
	}
	if(htim->Instance==TIM3)
	{
		F_R39=1000000.0/(HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1)+1.0);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM17)
	{
		if(tim_b3_con)
		{
			if(++tim500==500)
			{
				e2p_data[e2p_index]++;
				tim500=0;
				tim_ok=1;
			}
		}
		if(tim_b4_con)
		{
			if(++tim500==500)
			{
				e2p_data[e2p_index]--;
				tim500=0;
				tim_ok=1;
			}
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	I2CInit();
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim17);
	e2p_data[0]=e2prom_read(0);
	e2p_data[1]=e2prom_read(1);
	e2p_data[2]=e2prom_read(2);
	HAL_Delay(100);
  while (1)
  {
	  HAL_ADC_Start(&hadc1);
	  V_R37=HAL_ADC_GetValue(&hadc1)/4090.0*3.3;
	  HAL_ADC_Stop(&hadc1);
	  HAL_ADC_Start(&hadc2);
	  V_R38=HAL_ADC_GetValue(&hadc2)/4090.0*3.3;
	  HAL_ADC_Stop(&hadc2);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  key();lcd();
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

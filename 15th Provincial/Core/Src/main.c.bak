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
int PD=1000,PH=5000,PX=0,NDA,NDB,NHA,NHB;
double A,A_T,B,B_T,amax,amin,bmax,bmin;
u8 b1,b11,b2,b22,b3,b33,b4,b44;
u8 index;
int tim_1s,tim_1s_con,tim_3s;
u8 pha,phb;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		B=1000000.0/HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1)+PX;
		B_T=1.0/B*1000000.0;
		if(B>PH&&phb==0) 
		{
			NHB++;
			phb=1;
		}if(B<PH) phb=0;
		if(B>bmax) bmax=B;
		if(B<bmin) bmin=B;
	}
	if(htim->Instance==TIM2)
	{
		A=1000000.0/HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1)+PX;
		A_T=1.0/A*1000000.0;
		if(A>PH&&pha==0) 
		{
			NHA++;
			pha=1;
		}if(A<PH) pha=0;
		if(A>amax) amax=A;
		if(A<amin) amin=A;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM17)
	{
		if(tim_1s_con)
			if(++tim_1s==1000)
			{
				tim_1s=0;
				tim_1s_con=0;
				NDA=NDB=NHA=NHB=0;
			}
		if(++tim_3s==3000)
		{
			tim_3s=0;
			if(amax-amin>PD) NDA++;
			if(bmax-bmin>PD) NDB++;
			amax=amin=A;
			bmax=bmin=B;
		}
	}
}
void lcd(void)
{
	u8 t[21];
	if(lcd_mod==0)
	{
		sprintf((char *)t,"        DATA        ");
		LCD_DisplayStringLine(Line1, (unsigned char *)t);
		if(index==0)
		{
			if(A<1000) sprintf((char *)t,"     A=%.0fHz         ",A);
			if(A>=1000) sprintf((char *)t,"     A=%.2fKHz      ",A/1000.0);
			if(A<0) sprintf((char *)t,"     A=NULL         ");
			LCD_DisplayStringLine(Line3, (unsigned char *)t);
			if(B<1000) sprintf((char *)t,"     B=%.0fHz      ",B);
			if(B>=1000) sprintf((char *)t,"     B=%.2fKHz      ",B/1000.0);
			if(B<0) sprintf((char *)t,"     B=NULL         ");
			LCD_DisplayStringLine(Line4, (unsigned char *)t);
		}
		if(index==1)
		{
			if(A_T<1000) sprintf((char *)t,"     A=%.0fuS         ",A_T);
			if(A_T>=1000) sprintf((char *)t,"     A=%.2fmS      ",A_T/1000.0);
			if(A<0) sprintf((char *)t,"     A=NULL         ");
			LCD_DisplayStringLine(Line3, (unsigned char *)t);
			if(B_T<1000) sprintf((char *)t,"     B=%.0fuS         ",B_T);
			if(B_T>=1000) sprintf((char *)t,"     B=%.2fmS      ",B_T/1000.0);
			if(B<0) sprintf((char *)t,"     B=NULL         ");
			LCD_DisplayStringLine(Line4, (unsigned char *)t);
		}
	}
	if(lcd_mod==1)
	{
		sprintf((char *)t,"        PARA        ");
		LCD_DisplayStringLine(Line1, (unsigned char *)t);
		sprintf((char *)t,"     PD=%dHz        ",PD);
		LCD_DisplayStringLine(Line3, (unsigned char *)t);
		sprintf((char *)t,"     PH=%dHz        ",PH);
		LCD_DisplayStringLine(Line4, (unsigned char *)t);
		sprintf((char *)t,"     PX=%dHz        ",PX);
		LCD_DisplayStringLine(Line5, (unsigned char *)t);
	}
	if(lcd_mod==2)
	{
		sprintf((char *)t,"        RECD        ");
		LCD_DisplayStringLine(Line1, (unsigned char *)t);
		//NDA,NDB,NHA,NHB
		sprintf((char *)t,"     NDA=%d         ",NDA);
		LCD_DisplayStringLine(Line3, (unsigned char *)t);
		sprintf((char *)t,"     NDB=%d         ",NDB);
		LCD_DisplayStringLine(Line4, (unsigned char *)t);
		sprintf((char *)t,"     NHA=%d         ",NHA);
		LCD_DisplayStringLine(Line5, (unsigned char *)t);
		sprintf((char *)t,"     NHB=%d         ",NHB);
		LCD_DisplayStringLine(Line6, (unsigned char *)t);
	}
}
void key(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0) b1=1;
	else b1=0;
	if(b1==1&&b11==0)
	{
		b11=1;
		if(lcd_mod==1)
		{
			if(index==0) PD+=100;
			if(index==1) PH+=100;
			if(index==2) PX+=100;
		}
	}if(b1==0&&b11==1) b11=0;
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0) b2=1;
	else b2=0;
	if(b2==1&&b22==0)
	{
		b22=1;
		if(lcd_mod==1)
		{
			if(index==0) PD-=100;
			if(index==1) PH-=100;
			if(index==2) PX-=100;
		}
	}if(b2==0&&b22==1) b22=0;
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0) b3=1;
	else b3=0;
	if(b3==1&&b33==0)
	{
		b33=1;
		if(lcd_mod==1)
		{
			index++;
			if(index==3) index=0;
		}
		if(lcd_mod==0)
		{
			index++;
			if(index==2) index=0;
		}
		if(lcd_mod==2)
		{
			tim_1s=0;
			tim_1s_con=1;
		}
	}if(b3==0&&b33==1) 
	{
		b33=0;
		tim_1s_con=0;
		tim_1s=0;
	}
	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0) b4=1;
	else b4=0;
	if(b4==1&&b44==0)
	{
		b44=1;
		if(++lcd_mod==3) lcd_mod=0;
		index=0;
		LCD_Clear(Black);
	}if(b4==0&&b44==1) b44=0;
	
	if(PD<=100) PD=100;
	if(PD>=1000) PD=1000;
	if(PH<=1000) PH=1000;
	if(PH>=10000) PH=10000;
	if(PX<=-1000) PX=-1000;
	if(PX>=1000) PX=1000;
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
  MX_TIM17_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_Delay(300);
	amax=amin=A;
	bmax=bmin=B;
	HAL_Delay(300);
	NHA=NHB=0;
  while (1)
  {
	  lcd();key();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOC,(uint16_t)0xff00,1);
	  if(lcd_mod==0) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
	  if(A>PH) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
	  if(B>PH) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
	  if(NDA>=3||NDB>=3) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
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

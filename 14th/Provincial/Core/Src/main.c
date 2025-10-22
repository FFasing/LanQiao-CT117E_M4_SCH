/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
int B1=0,B2=0,B3=0,B4=0;
int b1=0,b2=0,b3=0,b4=0;
u8 B2_5s_Con=0;
int B2_5s=0;
u8 B4_2s_Con=0;
int B4_2s=0;
u16 led_state=0xffff;
u8 LD2_0_1s;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
u8 LCD_mod=0;
int    M=0;		//PWM输出模式
int    P=0;		//实时占空比
double V=0;		//实时速度
int	 R=1,r=1;
int	 K=1,k=1;
int	 N=0;		//PWM切换次数
double MH=0;	//高频最大速度
double ML=0;	//低频最大速度

u8 LD2_MODE=0;
u8 B2_Set_Index=0;
u8 B4_P_Lock=0;
double V_R37=0;
double PA7_Frq=0;

int T2_PSC=400-1;
int half_s;

double V_old=0;
int timer_2s;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_Write(int led,int mode)
{
	u16 temp=GPIOC->ODR;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	GPIOC->ODR=led_state;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8<<(led-1),mode^=1);
	led_state=GPIOC->ODR;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	GPIOC->ODR=temp;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM16)
	{
		if(++timer_2s==2000)
		{
			timer_2s=0;
			V_old=V;
		}
		if(B2_5s_Con==1)
		{
			if(++half_s==5000/200)
			{
				half_s=0;
				if(T2_PSC!=400-1&&M==0) T2_PSC+=200/200;
				if(T2_PSC!=200-1&&M==1) T2_PSC-=200/200;
				TIM2->PSC=T2_PSC;		
			}
			if(++LD2_0_1s==100)
			{
				LD2_0_1s=0;
				LD2_MODE^=1;
			}
			if(++B2_5s==5000) 
			{
				B2_5s=0;
				B2_5s_Con=0;
				LD2_MODE=0;
				LD2_0_1s=0;
			}
			LED_Write(2,LD2_MODE);
		}
		if(B4_2s_Con==1)
		{
			if(++B4_2s==2000) 
			{
				B4_2s=0;
				B4_2s_Con=0;
				B4_P_Lock^=1;
			}
		}
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LCD_Proc(u8 LCD_mod)
{
	char text[20];
			LCD_SetTextColor(Yellow);
			sprintf(text,"V:%.1f old:%.1f  ",V,V_old);
			LCD_DisplayStringLine(Line0, (u8*)text);
			LCD_SetTextColor(White);
	switch(LCD_mod)
	{
		case 0:
			sprintf(text,"        DATA      ");
			LCD_DisplayStringLine(Line1, (u8*)text);
			if(M==0) sprintf(text,"     M=L          ");
			if(M==1) sprintf(text,"     M=H          ");
			LCD_DisplayStringLine(Line3, (u8*)text);
			sprintf(text,"     P=%d%%       ",P);
			LCD_DisplayStringLine(Line4, (u8*)text);
			sprintf(text,"     V=%.1f       ",V);
			LCD_DisplayStringLine(Line5, (u8*)text);
		break;
		case 1:
			if(r<1) 	r=10;
			if(r>10) r=1;
			if(k<1) 	k=10;
			if(k>10) k=1;
			sprintf(text,"        PARA      ");
			LCD_DisplayStringLine(Line1, (u8*)text);
			sprintf(text,"     R=%d          ",r);
			LCD_DisplayStringLine(Line3, (u8*)text);
			sprintf(text,"     K=%d          ",k);
			LCD_DisplayStringLine(Line4, (u8*)text);
		break;
		case 2:
			sprintf(text,"        RECD      ");
			LCD_DisplayStringLine(Line1, (u8*)text);
			sprintf(text,"     N=%d          ",N);
			LCD_DisplayStringLine(Line3, (u8*)text);
			sprintf(text,"     MH=%.1f       ",MH);
			LCD_DisplayStringLine(Line4, (u8*)text);
			sprintf(text,"     ML=%.1f       ",ML);
			LCD_DisplayStringLine(Line5, (u8*)text);
		break;
	}
	
}
void KEY_Proc(void)
{
	B1=HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin);
	B2=HAL_GPIO_ReadPin(B2_GPIO_Port,B2_Pin);
	B3=HAL_GPIO_ReadPin(B3_GPIO_Port,B3_Pin);
	B4=HAL_GPIO_ReadPin(B4_GPIO_Port,B4_Pin);
	if(B1==0&&b1==1)
	{
		b1=0;
		LCD_mod++;
		if(LCD_mod>2) LCD_mod=0;
		if(LCD_mod==1) B2_Set_Index=0;
		if(LCD_mod==2)
		{
			R=r;
			K=k;
		}
		LCD_Clear(Black);
		timer_2s=1;
	}
	if(B1==1&&b1==0)
	{
		b1=1;
	}
	
	if(B2==0&&b2==1)
	{
		b2=0;
		if(LCD_mod==0&&B2_5s_Con==0)
		{
			B2_5s_Con=1;
			B2_5s=0;
			M^=1;
			N++;
		}
		if(LCD_mod==1) B2_Set_Index^=1;
	}
	if(B2==1&&b2==0)
	{
		b2=1;
	}
	
	if(B3==0&&b3==1)
	{
		b3=0;
		if(LCD_mod==1)
		{
			if(B2_Set_Index==0) r++;
			if(B2_Set_Index==1) k++;
		}
	}
	if(B3==1&&b3==0)
	{
		b3=1;
	}
	
	if(B4==0&&b4==1)
	{
		b4=0;
		if(LCD_mod==0)
		{
			B4_2s_Con=1;
			B4_2s=0;
		}
		if(LCD_mod==1)
		{
			if(B2_Set_Index==0) r--;
			if(B2_Set_Index==1) k--;
		}
	}
	if(B4==1&&b4==0)
	{
		b4=1;
		if(LCD_mod==0)
		{
			B4_2s_Con=0;
			B4_2s=0;
		}
	}
	
}
void LED_Proc(void)
{
	if(LCD_mod==0) LED_Write(1,1);
	else LED_Write(1,0);
	if(B4_P_Lock) LED_Write(3,1);
	else LED_Write(3,0);
}
void R37_Read(void)
{
	HAL_ADC_Start(&hadc2);
	V_R37=HAL_ADC_GetValue(&hadc2)/4095.0*3.3;
	HAL_ADC_Stop(&hadc2);
	if(B4_P_Lock==0)
	{
		P=37.5*V_R37-27.5;
		if(V_R37<=1.0) P=10;
		if(V_R37>=3.0) P=85;
	}
}
void Freq_Proc(void)
{
	//PA7_Frq=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2);
	PA7_Frq=TIM3->CCR2;
	PA7_Frq=1000000.0/(PA7_Frq+1.0);
	V=(PA7_Frq*2.0*3.14*R)/(100.0*K);
	if(V_old==V)
	{
		if(M==0&&ML<V&&PA7_Frq==2000) ML=V;
		if(M==1&&MH<V&&PA7_Frq==4000) MH=V;
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
  MX_TIM16_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	LCD_Clear(Black);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  TIM2->PSC=T2_PSC;
  //__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,50);
  TIM2->CCR2=50;
  HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_2);
  GPIOC->ODR=0xffff;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Freq_Proc();
	  R37_Read();
	  KEY_Proc();
	  LCD_Proc(LCD_mod);
	  LED_Proc();
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

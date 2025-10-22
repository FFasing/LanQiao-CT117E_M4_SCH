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
  * If no LICENSE file comes with this software, it is prad AS-IS.
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
u8 B1=0,B2=0,B3=0,B4=0;
u8 b1=0,b2=0,b3=0,b4=0;
int tim_2s=0;
int tim_2s_lock=0;
u16 led_state=0xffff;
int tim_01s=0;
int l1=0;
u8 select_index=0;

int F=0;	//PA1频率
int D=0;	//PA1占空比
double A=0;	//R37电压
double T=0;	//温度

int FH=2000;
double AH=3.0;
int TH=30;
int fh=0;
double ah=0;
int th=0;

int FN=0;
int AN=0;
int TN=0;

int FP=1; //分频系数
double VP=0.9; //回放最小值
int TT=6; //记录回放时间
int fp=0; //分频系数
double vp=0; //回放最小值
int tt=0; //记录回放时间
int tt_time=0;

u8 system_lock=0;	//0-默认 1-锁 2-电压 3-脉冲
u8 LCD_mode=0;

int F_Record[1000];
int D_Record[1000];
int D_Index=0,F_Index=0;
u8 Record_Flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_Write(int led,int mode)
{
	u16 temp;
	temp = GPIOC->ODR;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	GPIOC->ODR = led_state;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8<<(led-1),mode^=1);
	led_state = GPIOC->ODR;
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
	GPIOC->ODR = temp;
}
void SYSTEM_Init(void)
{
	tim_2s_lock=0;
	tim_2s=0;
	LED_Write(1,0);
	fh=FH=2000;
	ah=AH=3.0;
	th=TH=30;
	FN=0;
	AN=0;
	TN=0;
	fp=FP=1; //分频系数
	vp=VP=0.9; //回放最小值
	tt=TT=6; //记录回放时间
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM16)
	{
		if(tim_2s_lock)
		{
			if(++tim_2s==200)
			{
				tim_2s=0;
				SYSTEM_Init();
			}
		}
		if(++tim_01s==10)
		{
			tim_01s=0;
			if(system_lock==1) LED_Write(1,l1^=1);
			if(system_lock!=1) LED_Write(1,0);
			if(system_lock==2) LED_Write(2,l1^=1);
			if(system_lock!=2) LED_Write(2,0);
			if(system_lock==3) LED_Write(3,l1^=1);
			if(system_lock!=3) LED_Write(3,0);
		}
		if(system_lock)
		{
			if(system_lock==1)
			{
				F_Record[F_Index++]=F;
				D_Record[D_Index++]=D;
			}
			if(system_lock==2)
			{
				F=F_Record[F_Index++];
				D=D_Record[D_Index++];
			}
			if(++tt_time==TT*100)
			{
				tt_time=0;
				system_lock=0;
			}
		}
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LED_Proc(void)
{ 
	if(F>FH) LED_Write(4,1);
	else LED_Write(4,0);
	if(A>AH) LED_Write(5,1);
	else LED_Write(5,0);
	if(T>TH) LED_Write(6,1);
	else LED_Write(6,0);
}
void KEY_Proc(void)
{
	if(system_lock==1) return;
	
	B1=HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin);
	B2=HAL_GPIO_ReadPin(B2_GPIO_Port,B2_Pin);
	B3=HAL_GPIO_ReadPin(B3_GPIO_Port,B3_Pin);
	B4=HAL_GPIO_ReadPin(B4_GPIO_Port,B4_Pin);
	if(B1==0&&b1==1)
	{
		b1=0;
		if(++LCD_mode==4) LCD_mode=0;
		if(LCD_mode==1) 
		{
			fh=FH;
			ah=AH;
			th=TH;
		}
		if(LCD_mode==2) 
		{
			FH=fh;
			AH=ah;
			TH=th;
		}
		if(LCD_mode==3) 
		{
			fp=FP;
			vp=VP;
			tt=TT;
		}
		if(LCD_mode==0) 
		{
			FP=fp;
			VP=vp;
			TT=tt;
		}
		select_index=0;
		LCD_Clear(Black);
	}
	if(B1==1&&b1==0) b1=1;
	
	if(B2==0&&b2==1)
	{
		b2=0;
		if(LCD_mode==0)
		{
			system_lock=1;
			D_Index=0;
			F_Index=0;
			tt_time=0;
			Record_Flag=1;
		}
		if(LCD_mode==2)
		{
			FN=AN=TN=0;
		}
		if(LCD_mode==1||LCD_mode==3)
		{
			if(++select_index==3) select_index=0;
		}
		
	}
	if(B2==1&&b2==0) b2=1;
	
	if(B3==0&&b3==1)
	{
		b3=0;
		if(LCD_mode==0&&Record_Flag)
		{
			system_lock=2;
			D_Index=0;
			F_Index=0;
			tt_time=0;
		}
		if(LCD_mode==1)
		{
			if(select_index==0) fh+=1000;
			if(select_index==1) ah+=0.3;
			if(select_index==2) th+=1;
		}
		if(LCD_mode==3)
		{
			if(select_index==0) fp+=1;
			if(select_index==1) vp+=0.3;
			if(select_index==2) tt+=2;
		}
		tim_2s_lock=1;
	}
	if(B3==1&&b3==0) 
	{
		b3=1;
		tim_2s_lock=0;
		tim_2s=0;
	}
	
	if(B4==0&&b4==1)
	{
		b4=0;
		if(LCD_mode==0&&Record_Flag)
		{
			system_lock=3;
			D_Index=0;
			F_Index=0;
			tt_time=0;
		}
		if(LCD_mode==1)
		{
			if(select_index==0) fh-=1000;
			if(select_index==1) ah-=0.3;
			if(select_index==2) th-=1;
		}
		if(LCD_mode==3)
		{
			if(select_index==0) fp-=1;
			if(select_index==1) vp-=0.3;
			if(select_index==2) tt-=2;
		}
		tim_2s_lock=1;
	}
	if(B4==1&&b4==0) 
	{
		b4=1;
		tim_2s_lock=0;
		tim_2s=0;
	}
}
void LCD_Proc(u8 LCD_mode)
{
	char text[20];
	sprintf(text,"tim_2s=%d        ",tim_2s);
	LCD_DisplayStringLine(Line0, (u8*)text);
	switch(LCD_mode)
	{	
		case 0:
			sprintf(text,"        DATA     ");
			LCD_DisplayStringLine(Line1, (u8*)text);
			sprintf(text,"     F=%d        ",F);
			LCD_DisplayStringLine(Line3, (u8*)text);
			sprintf(text,"     D=%d        ",D);
			LCD_DisplayStringLine(Line4, (u8*)text);
			sprintf(text,"     A=%.1f      ",A);
			LCD_DisplayStringLine(Line5, (u8*)text);
			sprintf(text,"     T=%.1f      ",T);
			LCD_DisplayStringLine(Line6, (u8*)text);
		break;
		case 1:
			sprintf(text,"        PARA     ");
			LCD_DisplayStringLine(Line1, (u8*)text);
			sprintf(text,"     FH=%d        ",fh);
			LCD_DisplayStringLine(Line3, (u8*)text);
			sprintf(text,"     AH=%.1f      ",ah);
			LCD_DisplayStringLine(Line4, (u8*)text);
			sprintf(text,"     TH=%d        ",th);
			LCD_DisplayStringLine(Line5, (u8*)text);
		break;
		case 2:
			sprintf(text,"        RECD     ");
			LCD_DisplayStringLine(Line1, (u8*)text);
			sprintf(text,"     FN=%d        ",FN);
			LCD_DisplayStringLine(Line3, (u8*)text);
			sprintf(text,"     AN=%d        ",AN);
			LCD_DisplayStringLine(Line4, (u8*)text);
			sprintf(text,"     TN=%d        ",TN);
			LCD_DisplayStringLine(Line5, (u8*)text);
		break;
		case 3:
			sprintf(text,"        PARA     ");
			LCD_DisplayStringLine(Line1, (u8*)text);
			sprintf(text,"     FP=%d        ",fp);
			LCD_DisplayStringLine(Line3, (u8*)text);
			sprintf(text,"     VP=%.1f      ",vp);
			LCD_DisplayStringLine(Line4, (u8*)text);
			sprintf(text,"     TT=%d        ",tt);
			LCD_DisplayStringLine(Line5, (u8*)text);
		break;
	}
	
}
void Back(void)
{
	double d_t;
	if(system_lock==1)
	{
		HAL_ADC_Start(&hadc2);
		A=HAL_ADC_GetValue(&hadc2)/4095.0*3.3;
		HAL_ADC_Stop(&hadc2);
		d_t=(-900.0/(10.0*VP-33.0))*A+(1000.0*VP-330.0)/(10.0*VP-33.0);
		if(A<=VP) D=10;
		else if(A>=3.3) D=85;
		else D = (int)d_t;
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
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
    LCD_Init();
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);
	 LCD_Clear(Black);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim16);
  SYSTEM_Init();
  while (1)
  {
	  Back();
	  KEY_Proc();
	  if(fh<=1000) fh=1000;
	  if(fh>=10000) fh=10000;
	  if(ah<=0) ah=0;
	  if(ah>=3.3) ah=3.3;
	  if(th<=0) th=0;
	  if(th>=80) th=80;
	  if(fp<=1) fp=1;
	  if(fp>=10) fp=10;
	  if(vp<=0) vp=0;
	  if(vp>=3.3) vp=3.3;
	  if(tt<=2) tt=2;
	  if(tt>=10) tt=10;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LCD_Proc(LCD_mode);
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

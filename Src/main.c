/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <math.h>
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
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
		void user_pwm_setvalue(uint16_t value)	
{	
    TIM_OC_InitTypeDef sConfigOC;	
  	
    sConfigOC.OCMode = TIM_OCMODE_PWM1;	
    sConfigOC.Pulse = value;	
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;	
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);	
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  	
}

		void user_pwm_setvalue2(uint16_t value)	
{	
    TIM_OC_InitTypeDef sConfigOC;	
  	
    sConfigOC.OCMode = TIM_OCMODE_PWM1;	
    sConfigOC.Pulse = value;	
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;	
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);	
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  	
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
	hrtc.Instance = RTC;	
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;	
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;	
  if (HAL_RTC_Init(&hrtc) != HAL_OK)	
  {	
    Error_Handler();	
  }


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_FSMC_Init();
  MX_I2C2_Init();
  //MX_RTC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
     LCD_INIT();
 
	
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};
	
	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
	
	uint16_t HMC5883L_Addr = 0x1E; 
	uint8_t CRA = 0x70; 
	uint8_t CRB = 0xA0;
	int16_t Xval;
	int16_t Yval;
	
	uint8_t memadd = 0x01;
	HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x00,1,&CRA,1,100);
	HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x01,1,&CRB,1,100);
	
	uint8_t XMSB;
	uint8_t XLSB;
	uint8_t YMSB;
	uint8_t YLSB;
	
		// LCD PART
		// LCD PART
		LCD_DrawString(30,0,"pH Min:");
    LCD_DrawString(30,20,"Current pH:");
    LCD_DrawString(30,40,"pH Max:");

    LCD_DrawString(30,60,"Temp Min:");
    LCD_DrawString(30,80,"Current Temp:");
    LCD_DrawString(30,100,"Temp Max:");

    LCD_DrawString(30,120,"clarity Min:");
    LCD_DrawString(30,140,"Current clarity:");
    LCD_DrawString(30,160,"clarity Max:");
		

    LCD_DrawString(60,180,"Feeding Timer :");
		
		LCD_DrawString(80,200,"Hr");
		LCD_DrawString(160,200,"Min");
		
		int phmin = 5;
		int phmax = 7;
		char phminstring[3];
		char phmaxstring[3];
		sprintf(phminstring, "%2d", phmin);
		sprintf(phmaxstring, "%2d", phmax);
		
		LCD_DrawString(200,0,phminstring);
		LCD_DrawString(200,40,phmaxstring);
		
		int tempmin = 20;
		int tempmax=30;
		char tempminstring[3];
		char tempmaxstring[3];
		sprintf(tempminstring, "%2d", tempmin);
		sprintf(tempmaxstring, "%2d", tempmax);
		
		LCD_DrawString(200,60,tempminstring);
		LCD_DrawString(200,100,tempmaxstring);
		
		int claritymin = 4;
		int claritymax = 6;
		char clarityminstring[3];
		char claritymaxstring[3];
		sprintf(clarityminstring, "%2d", claritymin);
		sprintf(claritymaxstring, "%2d", claritymax);
		
		LCD_DrawString(200,120,clarityminstring);
		LCD_DrawString(200,160,claritymaxstring);
		
		
		int hr = 0;
		int min = 0;
		char hrstring[3];
		char minstring[3];
		sprintf(hrstring, "%2d", hr);
		sprintf(minstring, "%2d", min);
		
		LCD_DrawString(50,200,hrstring);
		LCD_DrawString(130,200,minstring);

		
		
		char Ahr[3];
		char Amin[3];
		char Asec[3];

		

    //triangle
		int hovercounter=0;
		int alarmd =0;
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADCEx_Calibration_Start(&hadc3);
	
	
	char decimal1[5], hexadecimal1[16];
	char decimal2[5], hexadecimal2[16];
	char decimal3[5], hexadecimal3[16];
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		GPIO_PinState k2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		GPIO_PinState k1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    
		
		//Alarm test
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		HAL_Delay(200);
		
		// Alarm function
		if(sAlarm.AlarmTime.Hours==sTime.Hours && sAlarm.AlarmTime.Minutes== sTime.Minutes && 
			sAlarm.AlarmTime.Seconds== sTime.Seconds){
				
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
				user_pwm_setvalue2(100);
				HAL_Delay(1200);
				user_pwm_setvalue2(200);
				alarmd =0;
		}
			
			
			
		
			// Alarm SET
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET){
				
				alarmd =1;
				sTime.Hours = 0;
				sTime.Minutes = 0;
				sTime.Seconds = 0;
			
				

				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
				{
					Error_Handler();
				}
				sAlarm.AlarmTime.Hours = 0;
				sAlarm.AlarmTime.Minutes = hr;
				sAlarm.AlarmTime.Seconds = min;
				sAlarm.Alarm = RTC_ALARM_A;
				if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
				{
					Error_Handler();
				}
				
		}
			
		if(alarmd==1 ){
				sprintf(Asec, "%2d", sTime.Seconds);
				sprintf(Amin, "%2d", sTime.Minutes);
				sprintf(Ahr, "%2d", sTime.Hours);
			
				LCD_DrawString(0,220,"Elapsed");
				LCD_DrawString(70,220,Ahr);
				LCD_DrawString(110,220,Amin);
				LCD_DrawString(150,220,Asec);
		}
			
	
			
		
			
				//JASPREET PART
			 
			 
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET){
		//LCD_DrawString(30,180,"works");
		hovercounter++;
		HAL_Delay(200);
		}
		switch(hovercounter % 8){
			case 0:
				Triangle(10,0,BLACK);
				Triangle(110,200,WHITE);
				break;
			case 1:
				Triangle(10,40,BLACK);
				Triangle(10,0,WHITE);
				
				break;
			case 2:
				Triangle(10,60,BLACK);
				Triangle(10,40,WHITE);
				break;
			case 3:
				Triangle(10,100,BLACK);
				Triangle(10,60,WHITE);
				break;
			case 4:
				Triangle(10,120,BLACK);
				Triangle(10,100,WHITE);
				break;
			
			case 5:
				Triangle(10,160,BLACK);
				Triangle(10,120,WHITE);
				break;
			case 6:
				Triangle(40,200,BLACK);
				Triangle(10,160,WHITE);
				break;
			
			case 7:
				Triangle(110,200,BLACK);
				Triangle(40,200,WHITE);
				break;

		
		}
		
			//setting the values k1 is for increase while k2 is for decrease
	
			switch(hovercounter%8){
			case 0:
				//changing the pH min
				if (k1==GPIO_PIN_SET){
					phmin+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					phmin-=1;
					HAL_Delay(400);
				}
				sprintf(phminstring, "%2d", phmin);
				LCD_DrawString(200,0,phminstring);
				break;
			case 1:
				if (k1==GPIO_PIN_SET){
					phmax+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					phmax-=1;
					HAL_Delay(400);
				}
				sprintf(phmaxstring, "%2d", phmax);
				LCD_DrawString(200,40,phmaxstring);
				break;
			case 2:
				if (k1==GPIO_PIN_SET){
					tempmin+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					tempmin-=1;
					HAL_Delay(400);
				}	
				sprintf(tempminstring,"%2d", tempmin );			
				LCD_DrawString(200,60,tempminstring);
				break;
			case 3:
				if (k1==GPIO_PIN_SET){
					tempmax+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					tempmax-=1;
					HAL_Delay(400);
				}
				sprintf(tempmaxstring,"%2d", tempmax);
				LCD_DrawString(200,100,tempmaxstring);
				break;
				
			
			case 4:
				if (k1==GPIO_PIN_SET){
					claritymin+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					claritymin-=1;
					HAL_Delay(400);
				}
				sprintf(clarityminstring,"%2d", claritymin);
				LCD_DrawString(200,120,clarityminstring);
				break;
			case 5:
				if (k1==GPIO_PIN_SET){
					claritymax+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					claritymax-=1;
					HAL_Delay(400);
				}
				sprintf(claritymaxstring,"%2d", claritymax);
				LCD_DrawString(200,160,claritymaxstring);
				break;
						
				
			case 6:
				if (k1==GPIO_PIN_SET){
					hr+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					hr-=1;
					HAL_Delay(400);
				}
				sprintf(hrstring,"%2d", hr);
				LCD_DrawString(50,200,hrstring);
				break;
			case 7:
				if (k1==GPIO_PIN_SET){
					min+=1;
					HAL_Delay(400);
				}
				if (k2==GPIO_PIN_SET){
					min-=1;
					HAL_Delay(400);
				}
				sprintf(minstring,"%2d", min);
				LCD_DrawString(130,200,minstring);
				break;
		
			}
			
		//GPIO
		HAL_ADC_Start(&hadc2);
		HAL_ADC_Start(&hadc3);
		HAL_ADC_Start(&hadc1);
			
		HAL_ADC_PollForConversion(&hadc2,200);
		HAL_ADC_PollForConversion(&hadc3,200);
		HAL_ADC_PollForConversion(&hadc1,200);

			
		uint32_t value_ADC2 =HAL_ADC_GetValue(&hadc2);
		float ph_value = value_ADC2;
		ph_value = (ph_value*3.3/4096)* -5.647 + 16.178;  //-5.9647 + 22.25    -5.697  + 16.178
			
		//float floatvalueph = (floatvalue*3.3/4096)* -5.697 + 16.178;
		
		//
		sprintf(decimal2,"%4f",ph_value);
		LCD_DrawString(200,20,decimal2);
			
		uint32_t value_ADC3=HAL_ADC_GetValue(&hadc3);
		float clarity_value = value_ADC3*10/4096;
		sprintf(decimal3,"%2f",clarity_value);
		LCD_DrawString(200,140,decimal3);
		
		
		uint32_t value_ADC1=HAL_ADC_GetValue(&hadc1);
		float temp_value = (value_ADC1*3300 /4096 ) /10.24;
		//(val/4096)* 3300/10.24
		sprintf(decimal1,"%2f",temp_value);
		LCD_DrawString(200,80,decimal1);
		
		
		
		
		

		
		// The Compass Part
		LCD_OpenWindow(0, 0, 320, 240);
		LCD_FillColor(100, WHITE);
		HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x02,1,&memadd,1,100); //write mode
		HAL_Delay(100);
		HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr<<1,0x03,1,&XMSB,1,100);
		HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr<<1,0x04,1,&XLSB,1,100);
		HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr<<1,0x07,1,&YMSB,1,100);
		HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr<<1,0x08,1,&YLSB,1,100);
		
		Xval= (XMSB<<8)+XLSB;
		Yval= (YMSB<<8)+YLSB;
		
		double X = Xval;
		double Y = Yval;
		
		double deg = atan(Y/X)*180/3.14;

	
	 if((X>=0) && (Y<0)) {
			deg= 360+deg;
		}
		else if((X<0) && (Y>=0)) {
			deg= 180+deg;
		} 
		else if((X<0) && (Y<0)) {
			deg= 180+deg;
		}
		
		char x[5];
		char xtry[5];
		char y[5];
		char degree[5];
		//sprintf(x , "%1f", X );
		//sprintf(y, "%1f", Y);
		sprintf(degree, "%1f", deg);
		//LCD_DrawString(8,230, "X");
		//LCD_DrawString(28,230, x);
		//LCD_DrawString(8, 250, "Y");
		//LCD_DrawString(28, 250, y);
		LCD_DrawString(16, 275, "degree:");
		LCD_DrawString(16, 300, degree);
		
		//boolean variables
		int checkph;
		checkph= (phmin<=ph_value && ph_value<=phmax);
		int checkclarity;
		checkclarity = (claritymin<=clarity_value && clarity_value<=claritymax);
		int checktemp;
		checktemp = (tempmin<=temp_value && tempmax>=temp_value);
		
		
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		
		
		if( !(checkph )) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		}
		if (!(checkclarity)){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		else {
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		}
		
		//
		if (!(checktemp)){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		}
		else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		}
		

		
		
		//stirrer
		if( !(checkph) || !(checkclarity )) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
		}
		
		//fan module
		if (!(checktemp)){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
			
		}
		else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		
		//pwm 2
		if (!(checkph)){
			
			user_pwm_setvalue(100);
			
				
		}else {
			user_pwm_setvalue(300);
			
		}
		
		
		//COMPASS BUZZER WORKS
		if(deg<270 && deg>220) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(GPIOB , GPIO_PIN_8 , GPIO_PIN_SET);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 45;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 10;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 280-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* USER CODE END TIM3_Init 2 */
  //HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 280-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
 HAL_TIM_MspPostInit(&htim4);
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END TIM4_Init 2 */
 // HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE5 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB5 PB8 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/*
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  
  UNUSED(hrtc);

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	user_pwm_setvalue(30);
	HAL_Delay(200);
	user_pwm_setvalue(30);

	
}
*/

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

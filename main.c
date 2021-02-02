/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main 452.c
  * @brief          : Main program body
  ******************************************************************************


  *******************************************************************************
  * File Name          : Patel_manankumar_assignment_3
  * Description        :
  * program takes input of MCP9700  Active Thermistor sensor which converts temperature into analog voltage
    converts it into digital value using ADC of STM32L432KC and then manipulate it using equation from
    datasheet and after calibration sends is out as output through 3 different means
    1)HD44780 LCD display
    2)Through virtual port over putty
    3)RGB led
    4)BUZZER
    ( this code is for is a pseudo temperature measuring system for a hospital so i decided to add a buzzer to beep when
    temperature is above or below safety range for safety related issues)
___________________________________________________________________________________________________
 __________________________________________________________________________________________________
  >> FUNCTIONS USED
    uint32_t measuredVolt(ADC_HandleTypeDef *hadc1);
    =>used to start,poll,and acquire ADC data
    =>returns: DIGITAL value of analog input given to ADC
    void set_rgb (uint8_t red, uint8_t green, uint8_t blue);
    =>used to manipulate timer 1 pwm generation to functions as a Pseudo analog.write() function
    =>input argument is set of 3 integer values in range of 0-255 representing brightness of
    color Red green and blue respectively.
    =>prescaler is set to (492-1) for 32Mhz clock
    =>counter mode is up mode
    void  LEDcontrol(float buffer);
    =>used to control RGB led
    =>input argument is a float value(ADC output converted to temperature in °C)
    void LCDcontrol(float buffer);
    =>used to print data on LCD display
    =>input argument is a float value(ADC output converted to temperature in °C)
    void vComControl(float buffer);
    =>used to display message on putty using Virtual port
    =>input argument is a float value(ADC output converted to temperature in °C)
    void alertBuzz();
    =>used to simulate buzzer with PA3 to produce alert tone on buzzer

  * Author:              Manankumarkumar patel
  * Date:                2020/03/22
  ******************************************************************************/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"stdio.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t measuredVolt(ADC_HandleTypeDef *hadc1);
void set_rgb (uint8_t red, uint8_t green, uint8_t blue);
void LEDcontrol(float buffer);
void LCDcontrol(float buffer);
void vComControl(float buffer);
void alaertBuzz();
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   int rate=0;
  while (1)
  {
	  if(rate<155)
  {
	 htim1.Instance->CCR1=rate;
	 rate=rate+1;
  }
     HAL_Delay(10);
     if(rate>=155)
     {
    	while(rate!=0)
    	{
        htim1.Instance->CCR1=rate;
        HAL_Delay(10);
    	rate--;
    	}
     }


    /* float convTemperature;   //converted temperature
     float refTemp=0.4;      //analog voltage value at 0°C (measured)
     float referanceVolt=3.3;//VDD for MCP9700  thermistor
     float resolution=4096;   //resolution is one part of 4096 in 12-bit ADC
     float tempCoffcient=0.01;//temperature coefficient of 10mV/°c as per datasheet
     float calibConst=2;//measured at 75 °F house temperature which had difference of almost 2°C when converted to °C
     uint32_t ADCdata;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
     /*ADCdata=measuredVolt(&hadc1); //ADC data data acquisition
     convTemperature=((((ADCdata*referanceVolt)/resolution)-refTemp)/tempCoffcient)+calibConst; //converting digital equivalent of temp into °C temperature to print
     LEDcontrol(convTemperature);//indicate color code of according to range given  through RGB led
     LCDcontrol(convTemperature);//printing converted temperature value on LCD display
     vComControl(convTemperature);//printing converted temperature value on putty through virtual port
*/

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

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  htim1.Init.Prescaler = 492-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
uint32_t measuredVolt(ADC_HandleTypeDef *hadc1)
{
  uint32_t measureVoltageADC=0;
  HAL_ADC_Start(hadc1);//instruction that starts ADC conversion
  HAL_ADC_PollForConversion(hadc1,200);//polling is done for conversion at interval of 200ms
  measureVoltageADC= HAL_ADC_GetValue(hadc1);//value acquired from adc is assigned to variable
  HAL_ADC_Stop(&*hadc1);//ADC is stopped
  return measureVoltageADC;//returning digial value
 }
void set_rgb (uint8_t red, uint8_t green, uint8_t blue)
{
  htim1.Instance->CCR1 = red;   //value of red is assigned to capture compare register value for ch1
  htim1.Instance->CCR2 = green; //value of red is assigned to capture compare register value for ch2
  htim1.Instance->CCR3 = blue;//value of red is assigned to capture compare register value for ch3
}
void LCDcontrol(float buffer)
{
  char StringBuffer[25]={0};
  if (buffer<20.5 && buffer>=(-40))
  {
    sprintf(StringBuffer,"LOW temp=%0.2fC",buffer);/*making a string to print by with float value integrated
    in it with precision of 2 digits after decimal*/
	HD44780_ClrScr();  //clears LCD screen
	HD44780_PutStr(StringBuffer);//a function that prints eah character of string given as argument
	void alertBuzz();
	HAL_Delay(200); //delay to avoid flickering incase of fast printing
  }
  else
  if (buffer>20.5 && buffer<37.3)
  {
    sprintf(StringBuffer,"Low tem=%0.2fC",buffer);
    HD44780_ClrScr();
    HD44780_PutStr(StringBuffer);
    void alertBuzz();
    HAL_Delay(1000);

  }
  else
  if (buffer>37.3 && buffer<39.5)
  {
    sprintf(StringBuffer,"NORM temp=%0.2fC",buffer);
    HD44780_ClrScr();
    HD44780_PutStr(StringBuffer);
    HAL_Delay(500);
  }
  else
  {
    sprintf(StringBuffer,"HIGH temp =%0.2fC",buffer);
    HD44780_ClrScr();
    HD44780_PutStr(StringBuffer);
    void alertBuzz();
    HAL_Delay(200);
  }
}
void vComControl(float buffer)
{
  if (buffer<20.5 && buffer>=(-40))
  { /*print function to print over putty serial terminal with alert message to check
   thermometer because this temp range is abnormal for human body*/
    printf("temperature very low check thermometer temperature=%1.2f°C\n",buffer);
  }
  else
  if (buffer>20.5 && buffer<37.3)
  {  //print function to print over putty serial terminal with alert message for low body temp
	printf("temperature low hypothermia alert..temperature=%1.2f °C\n",buffer);

  }
  else
  if (buffer>37.3 && buffer<39.5)
  {  //print function to print over putty serial terminal
	printf("temperature normal temperature=%1.2f°C\n ",buffer);
  }
  else
  {  //print function to print over putty serial terminal with alert message for fever
	printf("temperature high FEVER alert temperature=%1.2f°C",buffer);
  }

}
void LEDcontrol( float buffer)
{

  if (buffer<20.5 && buffer>=(-40))
  { //input represents intensities for Red green and blue light respectively
    set_rgb(0,0,255);//for BLUE light
  }
  else
  if (buffer>20.5 && buffer<37.3)
  {
    set_rgb(0,255,0);//for GREEN light
  }
  else
  if (buffer>37.3 && buffer<39.5)
  {
    set_rgb(255,120,0);//RGB intensity values for ORANGE light
  }
  else
  {
    set_rgb(255,0,0); //for RED;
  }
}
void alertBuzz()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);//turning buzzer on
	HAL_Delay(700);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);//turning buzzer off
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);//turning buzzer on
	HAL_Delay(700);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);//turning buzzer off
	HAL_Delay(300);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);//turning buzzer on
	HAL_Delay(700);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);//turning buzzer off
}
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

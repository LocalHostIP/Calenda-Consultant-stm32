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
#include "LCD.h"
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* LCD */
#define LCD_DATA_PORT 		GPIOB
#define LCD_RSE_PORT 		GPIOC
#define LCD_RS_PIN 			GPIO_PIN_8
#define LCD_E_PIN 			GPIO_PIN_6
#define LCD_LENGTH	 		15

LCD_PinType LCD_DATA_PINS[] = {GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15,GPIO_PIN_1};
LCD_Struct_t LCD;
//Text that goes to LCD when updating function is called
char LCD_Text[2][LCD_LENGTH+1];
//Reprint in LCD
void updateLCD();
//Update text in LCD
void updateLCDStrings(const char *s1,const char *s2);
//Flag to indicate when to update screen in super loop
int fUpdateLCD=0;

/* USART2 Variables */
#define LENGTH_RBUFFER 300
#define LENGTH_SBUFFER 1
unsigned char read_buffer[LENGTH_RBUFFER+1];
unsigned char send_buffer[LENGTH_SBUFFER+1];
void processBufferEvents();
void putLCDEvents();
//Flag to indicate when to update events in super loop
int fUpdateEvents=0;

//Events
//Struct to save summary and date of all events
typedef struct{
	unsigned char summary[LCD_LENGTH+1];
	unsigned char date[LCD_LENGTH+1];
}Events;

#define MAX_EVENTS 5
int c_maxEvent = 0;
int selected_event = 0;
//List of events
Events events[MAX_EVENTS];

/* Joystick */
//Middle val of joystick
#define MVAL_JOYSTICK 2000
uint32_t val_joystick;
uint32_t last_val_joystick;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* LCD INIT */
  HAL_Delay(1000);
  LCD = LCD_Create(LCD_DATA_PORT, LCD_DATA_PINS, LCD_RSE_PORT, LCD_RS_PIN, LCD_RSE_PORT, LCD_E_PIN);
  HAL_Delay(500);
  LCD_XY(&LCD,0,0);
  HAL_Delay(500);
  LCD_String(&LCD,"LOADING...");

  /* get events */
  fUpdateLCD=1;
  fUpdateEvents=1;

  //Start update events timmer
  HAL_TIM_Base_Start_IT(&htim2);
  //Start update LCD timmer
  HAL_TIM_Base_Start_IT(&htim3);

  //Start ADC for controls
  HAL_ADC_Start(&hadc1);
  val_joystick = 0;
  //Set las val to middle in joystick
  last_val_joystick = MVAL_JOYSTICK;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*Code for Joystick */
	  HAL_ADC_Start(&hadc1);
	  val_joystick = HAL_ADC_GetValue(&hadc1);

	  //Move up
	  if (last_val_joystick < MVAL_JOYSTICK+500 && val_joystick > MVAL_JOYSTICK+500){
		  selected_event++;
		  if (selected_event>=c_maxEvent)
			  selected_event=c_maxEvent-1;
		  putLCDEvents();
		  fUpdateLCD=1;
	  }
	  //Move down
	  else if (last_val_joystick > MVAL_JOYSTICK-500 && val_joystick < MVAL_JOYSTICK-500){
		  selected_event--;
		  if (selected_event<0)
			  selected_event=0;
		  putLCDEvents();
		  fUpdateLCD=1;
	  }
	  last_val_joystick=val_joystick;


	  /* Check for update flags */
	  if (fUpdateEvents){
		//Ask for info
		memset(send_buffer,0,LENGTH_SBUFFER);
		strcpy((char *) send_buffer,"e");
		HAL_UART_Transmit(&huart2,send_buffer,LENGTH_SBUFFER,5000);
		//Get info
		memset(read_buffer,0,LENGTH_RBUFFER);

		HAL_UART_DMAStop(&huart2);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, read_buffer, LENGTH_RBUFFER);
		//Disable half data transfered
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);

		fUpdateEvents=0;
	  }
	  if (fUpdateLCD){
		updateLCD();
		fUpdateLCD=0;
	  }

	  //Dummy code for button
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  }

	  HAL_Delay(50);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void processBufferEvents(){

	/*
	 * Save received buffer to array of structures of events
	* Data format: summary_1\tdate_1\tsummary_2\tdate_2...
	*/

	//Process data
	const int length_buffer = strlen((char *) read_buffer);
	unsigned char *start_summary,*start_date;
	unsigned char *end_date;
	int final_summary_index,final_date_index;

	//If data received
	if (strlen((char *) read_buffer)>0){
	  //start in read_buffer
	  start_summary = read_buffer;
	  for (c_maxEvent=0; c_maxEvent < MAX_EVENTS ;c_maxEvent++){
		  //Get starts of summary and date

		  //Check first character where \t appears, that is start of date
		  start_date = (unsigned char *) strchr((char *) start_summary+1,'\t');
		  //Check if start date was not found
		  if (start_date==0){
			  //If not found then data is not complete, terminate and do not add this event
			  c_maxEvent--;
			  if (c_maxEvent<0)
				  c_maxEvent=0;
			  break;
		  }

		  //Get end of date
		  end_date = (unsigned char *) strchr((char *) start_date+1,'\t');

		  //Get lengths
		  final_summary_index = (int)(start_date-start_summary);
			  //If end_date was found then calculate length
		  if (end_date!=0){
			  final_date_index = (int)(end_date - start_date);
		  }else{
			  //If end_date was found (\t), then end of date will be end of buffer
			  final_date_index = (int)(read_buffer+ length_buffer - start_date);
		  }

		  //Copy data to structure
			  //Copy summary
				  //Clean summary
		  memset(events[c_maxEvent].summary,0,LCD_LENGTH);
		  //Check to not pass over LCD_LENGTH
		  if(LCD_LENGTH < final_summary_index){
			  strncpy((char *) events[c_maxEvent].summary,(char *) start_summary,LCD_LENGTH);
		  }else{
			  strncpy((char *) events[c_maxEvent].summary,(char *) start_summary,final_summary_index);
		  }
			  //Copy date
		  memset(events[c_maxEvent].date,0,LCD_LENGTH);
		  if(LCD_LENGTH < final_date_index){
			  strncpy((char *) events[c_maxEvent].date,(char *) (start_date+1),LCD_LENGTH);
		  }else{
			  strncpy((char *) events[c_maxEvent].date,(char *) (start_date+1),final_date_index-1);
		  }

		  //Get next start for next summary
		  start_summary = end_date+1;

		  //If no new \t was found then terminate
		  if (end_date==0){
			  c_maxEvent++;
			  break;
		  }
	  }
	}
	else{
	  //If no data received
	  c_maxEvent=0;
	}
	//If selected event is greater than now updated max number of events received, then set selected to max
	if (selected_event>=c_maxEvent){
		if (c_maxEvent==0){
			selected_event=0;
		}else{
			selected_event=c_maxEvent-1;
		}
	}

	putLCDEvents();
}

void updateLCDStrings(const char *s1,const char *s2){
	//Clean LCD strings
	memset(LCD_Text[0],0,LCD_LENGTH+1);
	memset(LCD_Text[1],0,LCD_LENGTH+1);

	strcpy((char *) LCD_Text[0],s1);
	strcpy((char *) LCD_Text[1],s2);

	fUpdateEvents=1;
}

void updateLCD(){
	//Update LCD
	LCD_Command(&LCD, LCD_CLEAR_SCREEN);
	LCD_XY(&LCD,0,0);
	LCD_String(&LCD,LCD_Text[0]);
	LCD_XY(&LCD,1,0);
	LCD_String(&LCD,LCD_Text[1]);
}

void putLCDEvents(){
	//If got any events
	if (c_maxEvent>0){
		updateLCDStrings((char *) events[selected_event].summary,(char *) events[selected_event].date);
	}else{
		updateLCDStrings("No events","Received!");
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim3){
	 //Update only LCD
	 fUpdateLCD=1;
  }
  if (htim == &htim2 )
  {
	//Update Events and LCD
    fUpdateEvents=1;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance==USART2){
		//Procces data
		processBufferEvents();
	}
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

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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

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
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*-------------------------------- LCD ------------------------ */
#define LCD_DATA_PORT 		GPIOB
#define LCD_RSE_PORT 		GPIOC
#define LCD_RS_PIN 			GPIO_PIN_8
#define LCD_E_PIN 			GPIO_PIN_6
#define LCD_LENGTH	 		15

LCD_PinType LCD_DATA_PINS[] = {GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15,GPIO_PIN_1};
LCD_Struct_t LCD;
//Text that goes to LCD when updating function is called
char *LCD_Text[2];
//Reprint in LCD
void LCDUpdate();
void LCDPut(const char *s1,const char *s2);
void LCDPutEvents();
//If parameter is set to 1, only puts menu selector for yes and no (this improves visual)
void LCDPutDelete(int);
void LCDAnimationDown(char *s1,char *s2);
void LCDAnimationUp(char *s1,char *s2);
//Flag to indicate when to update screen in super loop
int fUpdateLCD=0;
#define LCD_DIVISION_LINE 		"  ___________  "
#define ANIAMATION_TIME 200
#define WITH_ANIMATION					0

//Delete Menu variables
const char DELETE_DIALOG[LCD_LENGTH+1] = "Delete event?";
const char delete_selector_diag_no[LCD_LENGTH+1] = "   Yes   (No)   ";
const char delete_selector_diag_yes[LCD_LENGTH+1] = "  (Yes)   No    ";
int onDeleteScreen = 0;
int no_selected = 1;

/*-------------------------------- USART ------------------------ */
void processBufferEvents();
#define LENGTH_RBUFFER 300
#define LENGTH_SBUFFER 2
#define TX_UPDATE 		'E';
#define TX_DELETE 		'D';
unsigned char read_buffer[LENGTH_RBUFFER+1];
unsigned char send_buffer[LENGTH_SBUFFER+1];

/*-------------------------------- Events ------------------------ */
#define MAX_EVENTS 5
//Struct to save summary and date of all events
typedef struct{
	char summary[LCD_LENGTH+1];
	char date[LCD_LENGTH+1];
}Events;
//List of events
Events events[MAX_EVENTS];

//Current amount of events received
int c_maxEvent = 0;
//Event selected to print on scren
int selected_event = 0;
//Flag to indicate when to update events in super loop
int fUpdateEvents=0;

void deleteEvent();

/*-------------------------------- Joystick ------------------------ */
#define JOYSTICK_MIN_CHANGE		1000
#define JOYSTICK_MED_VAL		2000

//Joystick val first index is up and down, second index is for left and right
//Set joystick previos val to middle val
uint32_t vals_joystick[2] = {JOYSTICK_MED_VAL,JOYSTICK_MED_VAL};
uint32_t last_vals_joystick[2] = {JOYSTICK_MED_VAL,JOYSTICK_MED_VAL};

//Flag that indicates if joystick has keep on the same enough time to make action again
int fTimmerJoystickUD = 0;
void joystickUp();
void joystickDown();
void joystickLeft();
void joystickRight();
void joystickCheckLR();
void joystickCheckUD();

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* LCD INIT */
  HAL_Delay(500);
  LCD = LCD_Create(LCD_DATA_PORT, LCD_DATA_PINS, LCD_RSE_PORT, LCD_RS_PIN, LCD_RSE_PORT, LCD_E_PIN);
  HAL_Delay(500);
  LCDPut("LOADING...","");
  LCDUpdate();

  /* get events */
  fUpdateEvents=1;
  //Start update events timmer
  HAL_TIM_Base_Start_IT(&htim2);
  //Start update LCD timmer
  HAL_TIM_Base_Start_IT(&htim3);
  LCD_XY(&LCD,0,0);
  HAL_ADC_Start_DMA(&hadc1,vals_joystick,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* --------------------- Joystick --------------- */
	  //If on delete event screen, cant move up and down to change selected event
	  if (onDeleteScreen==0){
		  joystickCheckUD();
	  }
	  joystickCheckLR();

	  /* --------------------- Check for delete event --------------- */

	  if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_RESET && onDeleteScreen){
		  if (no_selected){
			  onDeleteScreen=0;
			  LCDPutEvents();
			  LCDUpdate();
		  }else{
			  deleteEvent();
			  fUpdateEvents=1;
			  onDeleteScreen=0;
			  no_selected=1;
			  LCDPutEvents();
			  LCDUpdate();
		  }
	  }

	  /* --------------------- Check for update Flags --------------- */
	  //Update Events
	  if (fUpdateEvents){
		//Ask for info
		//Clean buffers
		memset(read_buffer,0,LENGTH_RBUFFER);

		send_buffer[0] = TX_UPDATE;
		send_buffer[1] = '-';

		fUpdateEvents=0;
		HAL_UART_Transmit_IT(&huart2,send_buffer,LENGTH_SBUFFER);
	  }
	  //Update LCD

	  if (fUpdateLCD){
		LCDUpdate();
		fUpdateLCD=0;
	  }

	  /* --------------------- Test --------------- */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
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
  htim2.Init.Period = 60999;
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
  htim3.Init.Period = 30999;
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
  htim4.Init.Prescaler = 9999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
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
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*-------------------------------- Events ------------------------ */

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

	LCDPutEvents();
	//Only change flag and no update because update can not be called on interrupts (Because of delays implemented on LCD file)
	fUpdateLCD=1;
}

void deleteEvent(){
	send_buffer[0] = TX_DELETE;
	send_buffer[1] = '0'+selected_event;
	HAL_UART_Transmit(&huart2,send_buffer, LENGTH_SBUFFER, 5000);
}

/*-------------------------------- LCD ------------------------ */

void LCDPut(const char *s1,const char *s2){
	LCD_Text[0] = s1;
	LCD_Text[1] = s2;
}

void LCDUpdate(){
	//Update LCD
	LCD_Command(&LCD, LCD_CLEAR_SCREEN);
	LCD_XY(&LCD,0,0);
	LCD_String(&LCD,LCD_Text[0]);
	LCD_XY(&LCD,1,0);
	LCD_String(&LCD,LCD_Text[1]);
}

void LCDPutEvents(){
	//If got any events
	if (onDeleteScreen==0){
		if (c_maxEvent>0){
			LCDPut((char *) events[selected_event].summary,(char *) events[selected_event].date);
		}else{
			LCDPut("No events","Received!");
		}
	}
}


void LCDAnimationDown(char *s1,char *s2){
	if (WITH_ANIMATION){
		//Animation to new text using already put text in LCD
		LCD_Text[1] = LCD_Text[0];
		LCD_Text[0] = LCD_DIVISION_LINE;
		LCDUpdate();
		HAL_Delay(ANIAMATION_TIME);

		LCD_Text[0] = s2;
		LCD_Text[1] = LCD_DIVISION_LINE;
		LCDUpdate();
		HAL_Delay(ANIAMATION_TIME);

		LCD_Text[0] = s1;
		LCD_Text[1] = s2;
		LCDUpdate();
	}else{
		LCD_Text[0] = s1;
		LCD_Text[1] = s2;
		LCDUpdate();
	}
}

void LCDAnimationUp(char *s1,char *s2){
	if (WITH_ANIMATION){
		//Animation to new text using already put text in LCD
		LCD_Text[0] = LCD_Text[1];
		LCD_Text[1] = LCD_DIVISION_LINE;
		LCDUpdate();
		HAL_Delay(ANIAMATION_TIME);

		LCD_Text[0] = LCD_DIVISION_LINE;
		LCD_Text[1] = s1;
		LCDUpdate();
		HAL_Delay(ANIAMATION_TIME);

		LCD_Text[0] = s1;
		LCD_Text[1] = s2;
		LCDUpdate();
	}else{
		LCD_Text[0] = s1;
		LCD_Text[1] = s2;
		LCDUpdate();
	}
}

void LCDPutDelete(int onlyUpdateMenu){
	if (onlyUpdateMenu){
		if (no_selected){
			LCD_Text[1]=(DELETE_DIALOG, delete_selector_diag_no);

		}else{
			LCD_Text[1]=(DELETE_DIALOG, delete_selector_diag_yes);
		}
		LCD_XY(&LCD,1,0);
		LCD_String(&LCD,LCD_Text[1]);
	}else{
		if (no_selected){
			LCDPut(DELETE_DIALOG, delete_selector_diag_no);
		}else{
			LCDPut(DELETE_DIALOG, delete_selector_diag_yes);
		}
	}
}

/*-------------------------------- Joystick ------------------------ */

void joystickCheckLR(){
	/*Code for Joystick */

	//Move Right
	if (last_vals_joystick[1] < JOYSTICK_MED_VAL + JOYSTICK_MIN_CHANGE && vals_joystick[1] > JOYSTICK_MED_VAL + JOYSTICK_MIN_CHANGE){
		joystickRight();
	}

	//Move left
	else if (last_vals_joystick[1] > JOYSTICK_MED_VAL - JOYSTICK_MIN_CHANGE && vals_joystick[1] < JOYSTICK_MED_VAL- JOYSTICK_MIN_CHANGE){
	  //Update selected event and update LCD text
		joystickLeft();
	}

	//Update last joystick val
	last_vals_joystick[1]=vals_joystick[1];
}

void joystickCheckUD(){
	/*Code for Joystick */

	//Move up
	if (last_vals_joystick[0] < JOYSTICK_MED_VAL + JOYSTICK_MIN_CHANGE && vals_joystick[0] > JOYSTICK_MED_VAL + JOYSTICK_MIN_CHANGE){
	  //Change Event selected and update lcd
	  joystickUp();

	  //Start Joystick timmer
	  HAL_TIM_Base_Stop(&htim4);
	  fTimmerJoystickUD=0;
	  HAL_TIM_Base_Start_IT(&htim4);
	}else if(vals_joystick[0] > JOYSTICK_MED_VAL + JOYSTICK_MIN_CHANGE && fTimmerJoystickUD){
	  //If keeps joystick up
	  joystickUp();

	  //Start Joystick timmer
	  HAL_TIM_Base_Stop(&htim4);
	  fTimmerJoystickUD=0;
	  HAL_TIM_Base_Start_IT(&htim4);
	}

	//Move down
	else if (last_vals_joystick[0] > JOYSTICK_MED_VAL - JOYSTICK_MIN_CHANGE && vals_joystick[0] < JOYSTICK_MED_VAL- JOYSTICK_MIN_CHANGE){
	  //Update selected event and update LCD text
	  joystickDown();

	  //Start Joystick timmer
	  HAL_TIM_Base_Stop(&htim4);
	  fTimmerJoystickUD=0;
	  HAL_TIM_Base_Start_IT(&htim4);
	}else if (vals_joystick[0] < JOYSTICK_MED_VAL - JOYSTICK_MIN_CHANGE && fTimmerJoystickUD){
	  //If joystick keeps down
	  joystickDown();

	  //Start Joystick timmer
	  HAL_TIM_Base_Stop(&htim4);
	  fTimmerJoystickUD=0;
	  HAL_TIM_Base_Start_IT(&htim4);
	}

	//Update last joystick val
	last_vals_joystick[0]=vals_joystick[0];
}

void joystickUp(){
	selected_event++;
	if (selected_event>=c_maxEvent)
	  selected_event=c_maxEvent-1;
	else
	  LCDAnimationDown(events[selected_event].summary,events[selected_event].date);
}

void joystickLeft(){
	if(c_maxEvent>0){
		if (onDeleteScreen){
			//If already on delete screen, move to left means select yes
			no_selected=0;
			LCDPutDelete(1);
			LCDUpdate();
		}else{
			//Change to delete screen and select no
			onDeleteScreen = 1;
			no_selected=1;
			LCDPutDelete(0);
			LCDUpdate();
		}
	}
}

void joystickRight(){
	if (onDeleteScreen){
		if (no_selected){
			//If already no selected means exit of delete screen
			onDeleteScreen = 0;
			LCDPutEvents();
		}else{
			no_selected = 1;
			LCDPutDelete(1);
		}
		LCDUpdate();
	}
}

void joystickDown(){
	selected_event--;
	if (selected_event<0)
	  selected_event=0;
	else
	  LCDAnimationUp(events[selected_event].summary,events[selected_event].date);
}

/*-------------------------------- Interrupts ------------------------ */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART2){
		//Get info
		//HAL_UART_DMAStop(&huart2);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, read_buffer, LENGTH_RBUFFER);
		//Disable half data transfereds
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //Timmer for LCD
  if (htim == &htim3){
	 //Update only LCD text
	 //fUpdateLCD=1;
  }
  //Timmer for Update events
  if (htim == &htim2 )
  {
	//Update Events
    fUpdateEvents=1;
  }

  //Timmer for Joystick
  if (htim == &htim4){
	  fTimmerJoystickUD=1;
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

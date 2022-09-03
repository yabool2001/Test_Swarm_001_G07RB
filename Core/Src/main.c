/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TX_TIMEOUT			100
#define RX_BUFF_SIZE			500
#define AT_COMM_TX_BUFF_SIZE	25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint16_t			checklist				= 0 ; //docelowo każdy bit będzie odpowiedzialny za kolejny krok aplikacji
uint8_t				waiting_for_answer		= 0 ;

char                hello[]         		= "Hello! Test_Swarm_001_G071RB\n" ;
char                good[]         			= "So far, so good !\n" ;
char                stm32_shutdown[]        = "STM32_Shutdown\n" ;
char                stm32_wakeup[]         	= "STM32 Wake Up\n" ;
HAL_StatusTypeDef   uart_status ;
uint8_t             rx_buff[RX_BUFF_SIZE] ;
char				tx_buff[AT_COMM_TX_BUFF_SIZE] ;

// SWARM AT Commands
const char 			cs_at_comm[]			= "$CS" ;
const char 			rt_0_at_comm[]			= "$RT 0" ;
const char			rt_q_rate_at_comm[]		= "$RT ?" ;
const char 			pw_0_at_comm[]			= "$PW 0" ;
const char			pw_q_rate_at_comm[]		= "$PW ?" ;
const char 			pw_mostrecent_at_comm[]	= "$PW @" ;
const char 			dt_0_at_comm[]			= "$DT 0" ;
const char			dt_q_rate_at_comm[]		= "$DT ?" ;
const char 			gs_0_at_comm[]			= "$GS 0" ;
const char			gs_q_rate_at_comm[]		= "$GS ?" ;
const char 			gj_0_at_comm[]			= "$GJ 0" ;
const char			gj_q_rate_at_comm[]		= "$GJ ?" ;
const char 			gn_0_at_comm[]			= "$GN 0" ;
const char			gn_q_rate_at_comm[]		= "$GN ?" ;
const char 			gn_mostrecent_at_comm[]	= "$GN @" ;
const char 			dt_mostrecent_at_comm[]	= "$DT @" ;
const char 			mt_del_all_at_comm[]	= "$MT D=U" ;
const char 			td_mzo_at_comm[]		= "$TD HD=300,\"MZO\"" ; // 5 minut na wysłanie wiadmości
const char 			sl_3ks_at_comm[]		= "$SL S=3000" ; // 50 minut spania dla Swarm
uint8_t				rt_unsolicited 			= 1 ;

// SWARM AT Answers
const char          cs_answer[]				= "$CS DI=0x" ;
const char          rt_ok_answer[]			= "$RT OK*22" ;
const char          rt_0_answer[]			= "$RT 0*16" ;
const char          pw_ok_answer[]			= "$PW OK*23" ;
const char          pw_0_answer[]			= "$PW 0*17" ;
const char          pw_mostrecent_answer[]	= "$PW " ;
const char          dt_ok_answer[]			= "$DT OK*34" ;
const char          dt_0_answer[]			= "$DT 0*00" ;
const char          gs_ok_answer[]			= "$GS OK*30" ;
const char          gs_0_answer[]			= "$GS 0*04" ;
const char          gj_ok_answer[]			= "$GJ OK*29" ;
const char          gj_0_answer[]			= "$GJ 0*1d" ;
const char          gn_ok_answer[]			= "$GN OK*2d" ;
const char          gn_0_answer[]			= "$GN 0*19" ;
const char          gn_mostrecent_answer[]	= "$GN " ;
const char 			mt_del_all_answer[]		= "$MT " ; // nie wiadomo ile ich będzie dlatego nie mogę ustawić "$MT 0*09"
const char 			td_ok_answer[]			= "$TD OK," ;
const char          sl_ok_answer[]			= "$SL OK*3b" ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void send2swarm_at_command ( const char* at_command , const char* answer , uint16_t step ) ;
void send2swarm_rt_0 () ;
void send2swarm_rt_query_rate () ;
uint8_t check_answer ( const char* s ) ;
uint8_t nmea_checksum ( const char *sz , size_t len ) ;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //HAL_RTCEx_DeactivateWakeUpTimer ( &hrtc ) ;
  __HAL_TIM_CLEAR_IT ( &htim6 , TIM_IT_UPDATE ) ; // żeby nie generować przerwania TIM6 od razu: https://stackoverflow.com/questions/71099885/why-hal-tim-periodelapsedcallback-gets-called-immediately-after-hal-tim-base-sta

  uart_status = HAL_UART_Transmit ( &huart2 , (const uint8_t *) hello , strlen ( hello ) , UART_TX_TIMEOUT ) ;
  HAL_UARTEx_ReceiveToIdle_DMA ( &huart1 , rx_buff , sizeof ( rx_buff ) ) ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while ( 1 )
  {
	  send2swarm_at_command ( cs_at_comm , cs_answer , 1 ) ;
	  if ( checklist == 1 )
		  send2swarm_at_command ( rt_0_at_comm , rt_ok_answer , 2 ) ;
	  if ( checklist == 2 )
	  	  send2swarm_at_command ( rt_q_rate_at_comm , rt_0_answer , 3 ) ; // Query RT rate
	  if ( checklist == 3 )
	  	send2swarm_at_command ( pw_0_at_comm , pw_ok_answer , 4 ) ;
	  if ( checklist == 4 )
	  	  send2swarm_at_command ( pw_q_rate_at_comm , pw_0_answer , 5 ) ;
	  if ( checklist == 5 )
	  	  send2swarm_at_command ( pw_mostrecent_at_comm , pw_mostrecent_answer , 6 ) ;
	  if ( checklist == 6 )
		  send2swarm_at_command ( dt_0_at_comm , dt_ok_answer , 7 ) ;
	  if ( checklist == 7 )
		  send2swarm_at_command ( dt_q_rate_at_comm , dt_0_answer , 8 ) ;
	  if ( checklist == 8 )
		  send2swarm_at_command ( gs_0_at_comm , gs_ok_answer , 9 ) ;
	  if ( checklist == 9 )
		  send2swarm_at_command ( gs_q_rate_at_comm , gs_0_answer , 10 ) ;
	  if ( checklist == 10 )
	  	  send2swarm_at_command ( gj_0_at_comm , gj_ok_answer , 11 ) ;
	  if ( checklist == 11 )
	  	  send2swarm_at_command ( gj_q_rate_at_comm , gj_0_answer , 12 ) ;
	  if ( checklist == 12 )
	  	  send2swarm_at_command ( gn_0_at_comm , gn_ok_answer , 13 ) ;
	  if ( checklist == 13 )
		  send2swarm_at_command ( gn_q_rate_at_comm , gn_0_answer , 14 ) ;
	  if ( checklist == 14 )
		  send2swarm_at_command ( gn_mostrecent_at_comm , gn_mostrecent_answer , 15 ) ;
	  if ( checklist == 15 )
		  send2swarm_at_command ( mt_del_all_at_comm , mt_del_all_answer , 16 ) ;
	  if ( checklist == 16 )
	  	  send2swarm_at_command ( td_mzo_at_comm , td_ok_answer , 17 ) ;
	  if ( checklist == 17 )
	  	  uart_status = HAL_UART_Transmit ( &huart2 , (const uint8_t *) good , strlen ( good ) , UART_TX_TIMEOUT ) ;
	  HAL_Delay ( 310000) ; // 5min. i 10 sekund obejmujące 5 minut na wysłanie wiadomości
	  send2swarm_at_command ( sl_3ks_at_comm , sl_ok_answer , 18 ) ; // Swarm sleep for 50 minutes
	  uart_status = HAL_UART_Transmit ( &huart2 , (const uint8_t *) stm32_shutdown , strlen ( stm32_shutdown ) , UART_TX_TIMEOUT ) ;
	  HAL_PWREx_EnterSHUTDOWNMode () ; // Enter the SHUTDOWN mode
	  uart_status = HAL_UART_Transmit ( &huart2 , (const uint8_t *) stm32_wakeup , strlen ( stm32_wakeup ) , UART_TX_TIMEOUT ) ;
	  checklist = 0 ;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 3600, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GREEN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback ( UART_HandleTypeDef *huart , uint16_t Size )
{
    if ( huart->Instance == USART1 )
    {
    	if ( rx_buff[0] != 0 )
    	{
    		// Jeśli dostałem potwierdzenie $RT = 0, to ustawiam odpowiednią zmienną
    		if ( strncmp ( (char*) rx_buff , rt_0_answer , strlen ( rt_0_answer ) ) == 0 )
    		{
    			rt_unsolicited = 0 ;
    			__NOP () ;
    		}
    		if ( strncmp ( (char*) rx_buff , rt_ok_answer , strlen ( rt_ok_answer ) ) == 0 )
    		{
    			__NOP () ;
    		}
    		//rx_buff[0] = 0 ;
    	}
    }
    HAL_UARTEx_ReceiveToIdle_DMA ( &huart1 , rx_buff , sizeof ( rx_buff ) ) ;
}
void send2swarm_rt_query_rate ()
{
	const char rt_q_rate_at_comm[] = "$RT ?" ;
	uint8_t cs = nmea_checksum ( rt_q_rate_at_comm , strlen ( rt_q_rate_at_comm ) ) ;
	char uart_tx_buff[10] ;

	sprintf ( (char*) uart_tx_buff , "%s*%02x\n" , rt_q_rate_at_comm , cs ) ;
	uart_status = HAL_UART_Transmit ( &huart1 , (const uint8_t *) uart_tx_buff ,  strlen ( (char*) uart_tx_buff ) , UART_TX_TIMEOUT ) ;
	waiting_for_answer = 1 ;
	HAL_TIM_Base_Start_IT ( &htim6 ) ;
		while ( waiting_for_answer )
		{
			if ( check_answer ( rt_0_answer ) )
			{
				checklist = 2 ;
				break ;
			}
		}
}
void send2swarm_rt_0 ()
{
	const char rt_0_at_comm[] = "$RT 0" ;
	uint8_t cs = nmea_checksum ( rt_0_at_comm , strlen ( rt_0_at_comm ) ) ;
	char uart_tx_buff[10] ;

	sprintf ( (char*) uart_tx_buff , "%s*%02x\n" , rt_0_at_comm , cs ) ;
	uart_status = HAL_UART_Transmit ( &huart1 , (const uint8_t *) uart_tx_buff ,  strlen ( (char*) uart_tx_buff ) , UART_TX_TIMEOUT ) ;
	waiting_for_answer = 1 ;
	HAL_TIM_Base_Start_IT ( &htim6 ) ;
	while ( waiting_for_answer )
	{
		/*
		 * Sprawdzić uważnie, bo przy "rt_ok_answer" poniższa funkcja wszystko puszczała
		 */
		if ( check_answer ( rt_ok_answer ) )
		{
			checklist = 1 ;
			break ;
		}
	}
}
void send2swarm_at_command ( const char* at_command , const char* answer , uint16_t step )
{
	uint8_t cs = nmea_checksum ( at_command , strlen ( at_command ) ) ;
	char uart_tx_buff[250] ;

	sprintf ( (char*) uart_tx_buff , "%s*%02x\n" , at_command , cs ) ;
	uart_status = HAL_UART_Transmit ( &huart1 , (const uint8_t *) uart_tx_buff ,  strlen ( (char*) uart_tx_buff ) , UART_TX_TIMEOUT ) ;
	waiting_for_answer = 1 ;
	HAL_TIM_Base_Start_IT ( &htim6 ) ;
	while ( waiting_for_answer )
	{
		if ( check_answer ( answer ) )
		{
			checklist = step ;
			break ;
		}
	}
}
uint8_t check_answer ( const char* answer )
{
	if ( strncmp ( (char*) rx_buff , answer , strlen ( answer ) ) == 0 )
	{
		rx_buff[0] = 0 ;
		return 1 ;
	}
	else
		return 0 ;
}
uint8_t nmea_checksum ( const char *sz , size_t len )
{
	size_t i = 0 ;
	uint8_t cs ;
	if ( sz [0] == '$' )
		i++ ;
	for ( cs = 0 ; ( i < len ) && sz [i] ; i++ )
		cs ^= ( (uint8_t) sz [i] ) ;
	return cs;
}
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
	if ( htim->Instance == TIM6 )
	{
		waiting_for_answer = 0 ;
		HAL_TIM_Base_Stop_IT ( &htim6 ) ;
		//NVIC_SystemReset () ; // Może kiedyś przyda się restartowanie aplikacji przy problemach z hardware
		__NOP () ;
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

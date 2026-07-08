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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FlashAddress 0x08007C00		//Page 31(Flash memory) initial address

#define ID1 0x1FE
#define ID2 0x1FD
#define ID3 0x1FC
#define ID4 0x1FB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
uint8_t icount = 0;
uint8_t period = 0;
uint8_t CAN_deviceID = 1;
uint8_t idx;
uint8_t CAN_Rxdata[8] = {0};
uint8_t CAN_Txdata[8];
uint16_t CPR;
uint8_t flag = 0;

/*
char str[64] = {0};
uint8_t DbgFlag = 0;
*/

void Drive_MTR(uint8_t stat, uint16_t duty) {
	if(duty > 999) duty = 999;

	switch (stat) {
	case 2:		//reverse
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
			break;
	case 3:		//forward
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
		break;
	default:	//free
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		break;
	}
}

void CAN_transmit(uint32_t TxID, uint8_t TXdata[]) {
	CAN_TxHeaderTypeDef TXHeader;
	uint32_t TxMailbox;
	if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan)) {
		TXHeader.StdId = TxID;
		TXHeader.RTR = CAN_RTR_DATA;
		TXHeader.IDE = CAN_ID_STD;
		TXHeader.DLC = 8;
		TXHeader.TransmitGlobalTime = DISABLE;

		HAL_CAN_AddTxMessage(&hcan, &TXHeader, TXdata, &TxMailbox);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim14) {	//TIM14 isr, 5Hz (0.2s)
		//ID indicate by blink LED
		static uint8_t num = 0;
		if (period <= CAN_deviceID * 2 - 1) {
			if (period % 2 == 0) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
			} else {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
			}
		}
		if(period == CAN_deviceID*2){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		}
		icount++;
		period++;
		if(flag){
			if(num % 2 == 0){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET);
			}else{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
			}
			if(num < 5){
				num++;
			}else{
				flag = 0;
				num = 0;
			}
		}
		if (period >= CAN_deviceID * 2 + 7)
			period = 0;
	}

	if (htim == &htim16) {	//TIM16 isr, 1kHz (1ms)
		//Transmit encoder data
		static int16_t pre_enc = 0;
		int16_t enc_cnt = (int16_t)TIM3->CNT;
		int16_t enc_deff, rpm;

		//calculate rpm
		/*if(pre_enc >= 0){
			enc_diff = (enc_cnt < 0) ? ((INT16_MAX - pre_enc) + (enc_cnt - INT16_MIN)) : (enc_cnt - pre_enc);
		}else{
			enc_diff = (enc_cnt >= 0) ? ((INT16_MAX - pre_enc) + (enc_cnt - INT16_MIN)) : (enc_cnt - pre_enc);
		}*/
		enc_deff = enc_cnt - pre_enc;

		rpm = enc_deff * (60000 / CPR);
		pre_enc = enc_cnt;

		CAN_Txdata[0] = enc_cnt >> 8;
		CAN_Txdata[1] = enc_cnt & 0xFF;

		CAN_Txdata[2] = (uint8_t)(rpm >> 8);
		CAN_Txdata[3] = (uint8_t)(rpm & 0xFF);

		CAN_Txdata[4] = 0;
		CAN_Txdata[5] = 0;

		CAN_Txdata[6] = (uint8_t)(CPR >> 8);
		CAN_Txdata[7] = (uint8_t)(CPR & 0xFF);
		CAN_transmit(0x20B + CAN_deviceID, CAN_Txdata);
		}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t data[8];
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
		uint16_t MessageID = RxHeader.StdId;
		if (CAN_deviceID <= 4 && MessageID == 0x1FE) {
			for (int i = 0; i < 8; i++) {
				CAN_Rxdata[i] = data[i];
			}
		}
		else if (CAN_deviceID >= 5 && CAN_deviceID <= 8 && MessageID == 0x1FD) {
			for (int i = 0; i < 8; i++) {
				CAN_Rxdata[i] = data[i];
			}
		}
		else if(CAN_deviceID >= 9 && CAN_deviceID <= 12 && MessageID == 0x1FC){
			for (int i = 0; i < 8; i++) {
				CAN_Rxdata[i] = data[i];
			}
		}
		else if(CAN_deviceID >= 13 && CAN_deviceID <= 16 && MessageID == 0x1FB){
			for (int i = 0; i < 8; i++) {
				CAN_Rxdata[i] = data[i];
			}
		}
		if((CAN_Rxdata[idx - 1] >> 4) == 0x0A){
			TIM3->CNT = 0;
			flag = 1;
		}
	}
}


void eraseFlash(uint32_t address) {
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = address;
	erase.NbPages = 1;

	uint32_t page_error = 0;
	HAL_FLASHEx_Erase(&erase, &page_error);
}

void writeFlash(uint32_t address, uint8_t *DATA) {
	HAL_FLASH_Unlock();
	eraseFlash(address);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *DATA);
	HAL_FLASH_Lock();
}

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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

//	setbuf(stdout, NULL);		//printf関数用

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);			//PWML
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);		//PHASE
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);	//PWMH

	//encoder resolution setting
	uint8_t DIP_SW = (uint8_t)(~GPIOA->IDR & 0x0f);

	switch (DIP_SW) {
		case 0:
			CPR = 8192;
			break;
		case 1:
			CPR = 4000;
			break;
		case 2:
			CPR = 3200;
			break;
		case 3:
			CPR = 1536;
			break;
		case 4:
			CPR = 4096;
			break;
		case 5:
			CPR = 2000;
			break;
		case 6:
			CPR = 1600;
			break;
		case 7:
			CPR = 768;
			break;
		case 8:
			CPR = 2048;
			break;
		case 9:
			CPR = 1000;
			break;
		case 10:
			CPR = 800;
			break;
		case 11:
			CPR = 384;
			break;
		case 12:
			CPR = 1024;
			break;
		case 13:
			CPR = 500;
			break;
		case 14:
			CPR = 400;
			break;
		case 15:
			CPR = 192;
			break;
	}

	uint16_t PWM = 0;
	uint8_t MTR_stat = 3;

	CAN_deviceID = *(uint8_t*) FlashAddress;

	if(CAN_deviceID <= 4) idx = CAN_deviceID * 2 - 1;
	else if(CAN_deviceID >= 5 && CAN_deviceID <= 8) 	idx = CAN_deviceID*2 - 9;
	else if(CAN_deviceID >= 9 && CAN_deviceID <= 12) 	idx = CAN_deviceID*2 - 17;
	else if(CAN_deviceID >= 13 && CAN_deviceID <= 16) 	idx = CAN_deviceID*2 - 25;

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//ID_setting
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {			//スイッチが押されたら
			HAL_TIM_Base_Stop_IT(&htim16);					//timer16の割り込みを停止
			Drive_MTR(0, 0);							//free
			CAN_deviceID = 0;
			icount = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET);		//LED1 ON
			while (1) {
				period = 1;			//LED0(PB1) OFF
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {	//スイッチが押されたら
					CAN_deviceID++;
					HAL_Delay(10);							//wait 10msチャタリング回避
					while (1) {
						if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {		//スイッチから手が離されたら
							if(CAN_deviceID > 16) CAN_deviceID = 1;
								icount = 0;
								HAL_Delay(10);					//wait 10msチャタリング回避
								break;
						}
					}
				}
				if (icount >= 8) {			//icount >= 8になったらループを終了
					if(CAN_deviceID <= 4) idx = CAN_deviceID * 2 - 1;
					else if(CAN_deviceID >= 5 && CAN_deviceID <= 8) 	idx = CAN_deviceID*2 - 9;
					else if(CAN_deviceID >= 9 && CAN_deviceID <= 12) 	idx = CAN_deviceID*2 - 17;
					else if(CAN_deviceID >= 13 && CAN_deviceID <= 16) 	idx = CAN_deviceID*2 - 25;
					writeFlash(FlashAddress, &CAN_deviceID);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);	//LED1 OFF
					icount = 0;
					period = 0;
					HAL_TIM_Base_Start_IT(&htim16);			//timer16の割り込み処
					break;
				}
			}
		}

		PWM = ((CAN_Rxdata[idx - 1] & 0x03) << 8) + CAN_Rxdata[idx];
		MTR_stat = (CAN_Rxdata[idx - 1] & 0x0C) >> 2;

		Drive_MTR(MTR_stat, PWM);
	}
    /* USER CODE END WHILE */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	CAN_FilterTypeDef filter;
	filter.FilterMode = CAN_FILTERMODE_IDLIST;
	filter.FilterIdHigh = ID1 << 5;			//ID1
	filter.FilterIdLow = ID2 << 5;			//ID2
	filter.FilterMaskIdHigh = ID3 << 5;		//ID3
	filter.FilterMaskIdLow = ID4 << 5;		//ID4
	filter.FilterScale = CAN_FILTERSCALE_16BIT;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterBank = 0;
	filter.SlaveStartFilterBank = 14;
	filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &filter);

  /* USER CODE END CAN_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 799;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 9999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 39;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	SHIELD_NOT_DETECTED = 0, SHIELD_DETECTED
} ShieldStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define B1SWITCH_SLICE_PUSH_TIME	2000
#define CHATTERING_REMOVE_TIME	10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_tx;
DMA_HandleTypeDef hdma_sdio_rx;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId sdReadTaskHandle;
osThreadId sdWriteTaskHandle;
osThreadId sdFormatTaskHandle;
osSemaphoreId sdReadBinarySemHandle;
osSemaphoreId sdWriteBinarySemHandle;
osSemaphoreId sdFormatBinarySemHandle;
/* USER CODE BEGIN PV */
FATFS SDFatFs; /* File system object for SD card logical drive */
FIL MyFile; /* File object */
char SDPath[4]; /* SD card logical drive path */
uint8_t workBuffer[2 * _MAX_SS];
//uint8_t buffSel;
uint16_t dataIdx;
uint16_t writeSize;
uint16_t writedataIdx;
uint8_t recvData;
uint16_t b1PushCounter;
char fileName[11];

uint32_t tickRec[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SDIO_SD_Init(void);
void StartDefaultTask(void const *argument);
void StartSdReadTask(void const *argument);
void StartSdWriteTask(void const *argument);
void StartSdFormatTask(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	 switch (GPIO_Pin)
	 {
	 case GPIO_PIN_6:
	 ITM_SendChar('6');
	 ITM_SendChar('\r');
	 ITM_SendChar('\n');
	 break;
	 case GPIO_PIN_7:
	 ITM_SendChar('7');
	 ITM_SendChar('\r');
	 ITM_SendChar('\n');
	 break;
	 case GPIO_PIN_10:
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	 break;
	 case GPIO_PIN_11:
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	 break;
	 case GPIO_PIN_14:
	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	 break;
	 }*/
	if (GPIO_Pin == GPIO_PIN_13)
	{
		b1PushCounter = B1SWITCH_SLICE_PUSH_TIME;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
	}
}

void ITM_SendStr(char *str)
{
	while (*str)
	{
		ITM_SendChar(*str);
		str++;
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
	MX_DMA_Init();
	MX_USART3_UART_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */
	b1PushCounter = 0;
	//  buffSel = 0;
	dataIdx = 0;
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of sdReadBinarySem */
	osSemaphoreDef(sdReadBinarySem);
	sdReadBinarySemHandle = osSemaphoreCreate(osSemaphore(sdReadBinarySem), 1);

	/* definition and creation of sdWriteBinarySem */
	osSemaphoreDef(sdWriteBinarySem);
	sdWriteBinarySemHandle = osSemaphoreCreate(osSemaphore(sdWriteBinarySem),
			1);

	/* definition and creation of sdFormatBinarySem */
	osSemaphoreDef(sdFormatBinarySem);
	sdFormatBinarySemHandle = osSemaphoreCreate(osSemaphore(sdFormatBinarySem),
			1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of sdReadTask */
	osThreadDef(sdReadTask, StartSdReadTask, osPriorityIdle, 0, 256);
	sdReadTaskHandle = osThreadCreate(osThread(sdReadTask), NULL);

	/* definition and creation of sdWriteTask */
	osThreadDef(sdWriteTask, StartSdWriteTask, osPriorityIdle, 0, 256);
	sdWriteTaskHandle = osThreadCreate(osThread(sdWriteTask), NULL);

	/* definition and creation of sdFormatTask */
	/*
	 osThreadDef(sdFormatTask, StartSdFormatTask, osPriorityIdle, 0, 256);
	 sdFormatTaskHandle = osThreadCreate(osThread(sdFormatTask), NULL);
	 */
	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 13;
	RCC_OscInitStruct.PLL.PLLN = 195;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 3;
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : USER_Btn_Pin PC6 PC7 */
	GPIO_InitStruct.Pin = USER_Btn_Pin | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PE10 PE11 PE14 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PG2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
	GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_VBUS_Pin */
	GPIO_InitStruct.Pin = USB_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
	/* USER CODE BEGIN 5 */
	/* init code for FATFS */
	//MX_FATFS_Init();
	/*## Register the file system object to the FatFs module ##############*/
	if (f_mount(&SDFatFs, (TCHAR const*) SDPath, 0) != FR_OK)
	{
		/* FatFs Initialization Error */
		Error_Handler();
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
	}
	/* Infinite loop */
	for (;;)
	{
		/*
		 HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		 osDelay(100);
		 */
		osDelay(1);
		if (b1PushCounter > 0)
		{
			if (--b1PushCounter
					< (B1SWITCH_SLICE_PUSH_TIME - CHATTERING_REMOVE_TIME))
			{
				if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)
						== GPIO_PIN_RESET)
				{
					//				  if(b1PushCounter < 2000){
					//					  osSemaphoreRelease(sdWriteBinarySemHandle);
					//				  }
					//				  else{
					osSemaphoreRelease(sdReadBinarySemHandle);
					//				  }
					b1PushCounter = 0;
				}
				else
				{
					if (b1PushCounter == 0)
					{
						osSemaphoreRelease(sdFormatBinarySemHandle);
					}
				}
			}
		}
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSdReadTask */
/**
 * @brief Function implementing the sdReadTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSdReadTask */
void StartSdReadTask(void const *argument)
{
	/* USER CODE BEGIN StartSdReadTask */
	FRESULT res; /* FatFs function common result code */
	uint32_t bytesread; /* File write/read counts */
	uint16_t fileSize;
	uint16_t readSize;
	uint16_t readDataIdx;
	uint16_t buffIdx;
	uint16_t sendBuffIdx;
	HAL_StatusTypeDef status;

	osSemaphoreWait(sdReadBinarySemHandle, 0);
	/* Infinite loop */
	for (;;)
	{
		osSemaphoreWait(sdReadBinarySemHandle, osWaitForever);

		/*## Open the text file object with read access ###############*/
		f_open(&MyFile, "STM32.TXT", FA_WRITE | FA_CREATE_ALWAYS);

		/*if (f_open(&MyFile, "STM32.TXT", FA_READ | FA_WRITE | FA_CREATE_ALWAYS)
		 != FR_OK)
		 {*/
		/* 'STM32.TXT' file Open for read Error */
		/*Error_Handler();
		 }*/
		/*else
		 {*/
		/*## Read data from the text file ###########################*/
		fileSize = MyFile.fsize; //.obj.objsize;
		readDataIdx = 0;
		buffIdx = 0;
		tickRec[3] = HAL_GetTick();
		do
		{
			readSize =
					((fileSize - readDataIdx) > _MAX_SS) ?
							_MAX_SS : fileSize - readDataIdx;
			tickRec[5] = HAL_GetTick();
			res = f_read(&MyFile, &workBuffer[buffIdx], readSize,
					(UINT*) &bytesread);
			tickRec[6] = HAL_GetTick();
			if ((bytesread == 0) || (res != FR_OK))
			{
				/* 'STM32.TXT' file Read or EOF Error */
				Error_Handler();
			}
			else
			{

			}
			buffIdx += readSize;
			if (buffIdx > _MAX_SS)
			{
				buffIdx = 0;
				sendBuffIdx = _MAX_SS;
			}
			else
			{
				buffIdx = _MAX_SS;
				sendBuffIdx = 0;
			}
			/*
			 do
			 {
			 status = HAL_UART_Transmit_IT(&huart6,
			 &workBuffer[sendBuffIdx], readSize);
			 } while (status == HAL_BUSY);
			 */
			readDataIdx += readSize;
		} while (readDataIdx < fileSize);
		tickRec[4] = HAL_GetTick();
		/*## Close the open text file #############################*/
		f_close(&MyFile);
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		/*}*/
		osDelay(1);
	}
	/* USER CODE END StartSdReadTask */
}

/* USER CODE BEGIN Header_StartSdWriteTask */
/**
 * @brief Function implementing the sdWriteTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSdWriteTask */
void StartSdWriteTask(void const *argument)
{
	/* USER CODE BEGIN StartSdWriteTask */
	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten; /* File write/read counts */
	char tmpFileName[] = "1234567890\0";
	osSemaphoreWait(sdWriteBinarySemHandle, 0);
	/* Infinite loop */
	for (;;)
	{
		osSemaphoreWait(sdWriteBinarySemHandle, osWaitForever);

		tickRec[0] = HAL_GetTick();

		strncpy(tmpFileName, (char*) &workBuffer[writedataIdx], 10);
		if (strstr(tmpFileName, ".txt") != NULL)
		{
			/*## Create and Open a new text file object with write access #####*/
			strcpy(fileName, tmpFileName);
			res = f_open(&MyFile, fileName, FA_CREATE_ALWAYS | FA_WRITE);
		}
		else
		{
			res = f_open(&MyFile, fileName, FA_OPEN_ALWAYS | FA_WRITE);
		}
		if (res != FR_OK)
		{
			/* 'STM32.TXT' file Open for write Error */
			Error_Handler();
		}
		else
		{
			/*## Write data to the text file ################################*/
			tickRec[1] = HAL_GetTick();
			res = f_write(&MyFile, &workBuffer[writedataIdx], writeSize,
					(void*) &byteswritten);
			tickRec[2] = HAL_GetTick();
			if ((byteswritten == 0) || (res != FR_OK))
			{
				/* 'STM32.TXT' file Write or EOF Error */
				Error_Handler();
			}
			else
			{
				/*## Close the open text file #################################*/
				f_close(&MyFile);
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			}
		}
		osDelay(1);
	}
	/* USER CODE END StartSdWriteTask */
}

/* USER CODE BEGIN Header_StartSdFormatTask */
/**
 * @brief Function implementing the sdFormatTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSdFormatTask */
void StartSdFormatTask(void const *argument)
{
	/* USER CODE BEGIN StartSdFormatTask */
	osSemaphoreWait(sdFormatBinarySemHandle, 0);
	/* Infinite loop */
	for (;;)
	{
		osSemaphoreWait(sdFormatBinarySemHandle, osWaitForever);

		/*## Create a FAT file system (format) on the logical drive #########*/
		/* WARNING: Formatting the uSD card will delete all content on the device */
		if (f_mkfs((TCHAR const*) SDPath, 1, sizeof(workBuffer)) != FR_OK)
		{
			/* FatFs Format Error */
			Error_Handler();
		}
		else
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}

		osDelay(1);
	}
	/* USER CODE END StartSdFormatTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM9 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM9)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./libs/printf.h"
#include "./libs/DS28CM00ID/DS28CM00ID.h"
#include "usart.h"
#include "i2c.h"
#include "sdmmc.h"
#include "fatfs.h"
#include "spi.h"
#include "./libs/ADC/ads124S0.h"
#include "./libs/ADC/ads123S0_enums.h"
#include "./libs/ADC/ads123S0_structs.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
	osThreadAttr_t defaultTask_attributes = {0};
	defaultTask_attributes.name = "defaultTask";
	defaultTask_attributes.priority = (osPriority_t) osPriorityNormal;
	defaultTask_attributes.stack_size = 8196 * 4;
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}


double toVoltage(int32_t val)
{
	return (val / 8388608.0) * 3.3; // divide by 2^23 and multiply by vref
}


#define HEX_BYTE	"#04X"
#define HEX_SHORT	"#06X"
#define HEX_WORD	"#010lX"
#define HEX_LONG	"#018llX"
#define SD_ASSERT(a,b,msg) if(a != b){ \
		size = sprintf(buffer, "SD test aborted, cause: %s\r\n", msg);\
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);\
		goto sd_end;\
} \


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	char buffer[1024] = {0};
	int size = 0;
	size = sprintf(buffer, "End-of-Line testing started, if you see this UART1 is working\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	size = sprintf(buffer, "LED test: \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	size = sprintf(buffer, "LEDs all off\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET);
	osDelay(500);

	size = sprintf(buffer, "LEDs all on\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_SET);
	osDelay(500);

	size = sprintf(buffer, "LED Red\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_R_Pin, GPIO_PIN_SET);
	osDelay(500);

	size = sprintf(buffer, "LED Blue\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_B_Pin, GPIO_PIN_SET);
	osDelay(500);

	size = sprintf(buffer, "LED Green\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_SET);
	osDelay(500);

	size = sprintf(buffer, "=======================================\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	size = sprintf(buffer, "Trying to read serial number....\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	DS28CM00_ID id1(&hi2c1);
	auto fam = id1.getFamily();
	auto mode = id1.getMode();
	auto sernum = id1.getID();
	size = sprintf(buffer, "ID Family: %" HEX_BYTE "\r\n"
							"ID Mode: %s\r\n"
							"ID Serial Number: %" HEX_LONG "\r\n", fam, mode == DS28CM00_ID::MODE::I2C ? "I2C" : "SMB" , sernum);
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);


	size = sprintf(buffer, "=======================================\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	size = sprintf(buffer, "Testing Black Body connection (UART2)\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	if(HAL_GPIO_ReadPin(CALIB_MODE_GPIO_Port, CALIB_MODE_Pin) == GPIO_PIN_SET)
	{
		// Device is in normal mode
		size = sprintf(buffer, "Did not detect the calibration rig, but will try sending anyway\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	}
	else
	{
		// Device is in calibration mode
		size = sprintf(buffer, "Calibration rig detected\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	}

	size = sprintf(buffer, "Black body set to 25*C\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, size, -1);

	size = sprintf(buffer, "SG1\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, size, -1);

	size = sprintf(buffer, "DA%.1f\n", 25.0);
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, size, -1);

	size = sprintf(buffer, "=======================================\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	if(HAL_GPIO_ReadPin(SDMMC1_CD_GPIO_Port, SDMMC1_CD_Pin) == GPIO_PIN_SET)
	{
		// No SD card detected
		size = sprintf(buffer, "SD card not detected, skipping test\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	}
	else
	{
		// SD card detected
		size = sprintf(buffer, "SD card detected, running filesystem tests\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

		MX_FATFS_Init();
		const char *path = "/";
		const char *name = "EOL_test.bin";

		FATFS fs;  // file system
		FIL fil; // File
		FRESULT fresult;  // result
		UINT bw;  // File read/write count

		// Mount
		fresult = f_mount(&fs, path, 1);	// mount partition
		SD_ASSERT(fresult, FR_OK, "Failed to mount partition");

		fresult = f_open(&fil, name,
			FA_WRITE | FA_OPEN_ALWAYS | FA_READ); // Open (or create) file
		SD_ASSERT(fresult, FR_OK, "Failed to open test file");

		for(int idx = 0; idx < 1025; idx++)
		{
			buffer[idx % 512] = (char) idx % 256;
			if(idx % 512 == 0 && idx > 0)
			{
				fresult = f_write(&fil, buffer, 512, &bw);
				if(fresult != FR_OK || bw != 512)
				{
					size = sprintf(buffer, "Failed writing to test file\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
					goto sd_end;
				}
			}
		}
		f_sync(&fil);
		size = sprintf(buffer, "write test complete, testing read\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
		for(int idx = 0; idx < 1024; idx+=512)
		{
			fresult = f_read(&fil, buffer, 512, &bw);
			if(fresult != FR_OK || bw != 512)
			{
				size = sprintf(buffer, "Failed reading from test file\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
				goto sd_end;
			}
			for(int idx2 = 0; idx2 < bw; idx2++)
			{
				if(buffer[idx2] != (idx2 % 256))
				{
					size = sprintf(buffer, "Read back data incorrect\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
					goto sd_end;
				}
			}
		}
		size = sprintf(buffer, "read test complete\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

		fresult = f_close(&fil);
		if(fresult != FR_OK)
		{
			size = sprintf(buffer, "failed closing the file\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
		}

		fresult = f_unlink(name);
		if(fresult != FR_OK)
		{
			size = sprintf(buffer, "failed deleting the file\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
		}
	}
sd_end:
	size = sprintf(buffer, "=======================================\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);


	size = sprintf(buffer, "Testing ADC, MAY HANG ON FAILURE!!\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	ADS124S0 adc(&hspi1, ADC_RST_Pin, ADC_RST_GPIO_Port, ADC_CS_Pin,
		ADC_CS_GPIO_Port,
		ADC_DRDY_Pin, ADC_DRDY_GPIO_Port, ADC_Start_Pin,
		ADC_Start_GPIO_Port);

	if (!adc.init())
	{
		size = sprintf(buffer, "Failed to initialize ADC\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
	}
	else
	{
		size = sprintf(buffer, "ADC did not fail initialization, attempting to read channels\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

		adc.setExcitationMagnitude(ADS124S0::EXCITATION_CURRENT::_10uA);
		adc.routeExcitation1(10); // route the excitation current to the thermistor
		adc.routeExcitation2(11);

		for(int idx = 0; idx < 12; idx++)
		{
			auto raw = adc.readChannel(idx);
			auto volt = toVoltage(raw);
			size = sprintf(buffer, "ADC channel %d voltage=%f\r\n", idx, volt);
			HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);
		}
	}
	size = sprintf(buffer, "=======================================\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);

	size = sprintf(buffer, "Testing complete, cycling colors\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, 100);



  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED_R_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(GPIOD, LED_B_Pin | LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED_B_Pin, GPIO_PIN_SET);
		osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

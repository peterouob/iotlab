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
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "string.h"
#include "stdio.h"
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
uint8_t recvBuf[2];

//B11117054
uint8_t tts_data_0[] = {0x5b,0x76,0x36,0x5d,0x5b,0x6d,0x30,0x5d,0x5b,0x74,0x35,0x5d, 0x42,0x31,0x31,0x31,0x31,0x37,0x30,0x35,0x34};
//customized
uint8_t tts_data_1[] = {/*0x5b,0x76,0x36,0x5d,0x5b,0x6d,0x30,0x5d,0x5b,0x74,0x35,0x5d,*/ 0xa6,0xd1,0xa4,0xd1,0xab,0x4f,0xa6,0xf6,0xaa,0xf7,0xa4,0x73,0xbb,0xc8,0xa4,0x73,0xa5,0xfe,0xb3,0xa3,0xa6,0xb3,0x20,0xa6,0xd1,0xa4,0xd1,0xb1,0xd0,0xad,0xf6,0xa7,0x4f,0xba,0xde,0xa6,0xbf,0xb4,0xf2,0xc0,0x73,0xaa,0xea,0xb0,0xab}; //Big5
	
QueueHandle_t queueMode;
	
TaskHandle_t handleSYN6288;
SemaphoreHandle_t xSem;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void SYN_FrameInfo(uint8_t music, uint8_t *data,uint8_t dataLength)
{
	
	uint8_t frame[50];
	uint8_t ecc = 0;
	
	//setting
	frame[0] = 0xfd;
	frame[1] = 0x00;
	frame[2] = dataLength + 3;
	frame[3] = 0x01;
	/*
	0x00 -> GB2312
	0x01 -> GBK
	0x02 -> BIG5
	*/
	frame[4] = 0x02 | music << 4;
	
	//ecc calculate
	for(int i = 0;i<5;i++)
	{
		ecc = ecc ^ (frame[i]);
	}
	
	for(int i = 0;i<dataLength;i++)
	{
		ecc = ecc ^ (data[i]);
	}
	
 
	//send msg to syn 6288
	memcpy(&frame[5], data, dataLength);
	frame[5 + dataLength] = ecc;
	HAL_UART_Transmit_DMA(&huart6, frame, 5 + dataLength + 1);
}

void SYN_SET(uint8_t *setting)
{
	uint8_t cmdLen;
	cmdLen = strlen((char*)setting);
	HAL_UART_Transmit_DMA(&huart6, setting, cmdLen);
}
	
void taskSYN6288(void *pvParm){
	uint8_t mode = 0;
	while (1){
		
		
		xQueueReceive(queueMode, &mode, NULL);
    BaseType_t queueModeReceive = xQueueReceive(queueMode, &mode, NULL);
		
		if(queueModeReceive == pdPASS){
			if (mode == 1){
				HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, 1);
				SYN_FrameInfo(0, tts_data_0, sizeof(tts_data_0));
				vTaskDelay(3000 / portTICK_PERIOD_MS);
				HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, 0);
			}else if (mode == 2){
				HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, 1);
				SYN_FrameInfo(0, tts_data_1, sizeof(tts_data_1));
				vTaskDelay(8000 / portTICK_PERIOD_MS);
				HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, 0);
			}else {
				vTaskDelay(200 / portTICK_PERIOD_MS);
			}
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		uint8_t mode = 0;
	
    switch (GPIO_Pin)
    {
    // sw1 PE3
    case GPIO_PIN_3:
    {
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == 1)
        {
					mode = 1;
					xQueueSendFromISR(queueMode, &mode, NULL);
        }
        break;
    }
    // sw2 PE4
    case GPIO_PIN_4:
    {
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 1)
        {
					mode = 2;
					xQueueSendFromISR(queueMode, &mode, NULL);
        }
        break;
    }
    default:
        break;
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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	queueMode = xQueueCreate(3, sizeof(uint8_t));
	xTaskCreate(taskSYN6288, "TTS Service", 1024, NULL, 1, &handleSYN6288);
	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

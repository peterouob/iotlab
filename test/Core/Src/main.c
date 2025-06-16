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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dht11.h"
#include "lcd.h"
#include "lcd_init.h"

#include "gps_neo6.h"
#include "max30102_for_stm32_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define UART_BUFFER_SIZE 256
#define BC_INTERFACE_MSG_LEN 11

uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_data;
volatile uint16_t uart_rx_index = 0;
volatile uint8_t uart_data_ready = 0;

volatile uint8_t lora_tx_ready = 1;

NEO6_State GpsData;
max30102_t max30102;

uint32_t max_heart_rate = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void taskST7789(void *pvParm){
	LCD_Fill(0, 0, 240, 240, WHITE);
	
	uint8_t str_id[] = "B11117054";
	uint8_t str_name[] = "Hsu Zhi-Dong";
	uint8_t str_phone[] = "0902-396-782";

	LCD_ShowString(10, 100, str_id, WHITE, BLACK, 32, 0);
	LCD_ShowString(10, 135, str_name, WHITE, BLACK, 32, 0);
	LCD_ShowString(10, 170, str_phone, WHITE, BLACK, 32, 0);
	
	while(1){
		vTaskDelay(5000 / portTICK_RATE_MS);
	}
}

void taskLora(void *pvParm)
{
	HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
	uint16_t message_len = 15;
	uint8_t message[message_len];
	memset(message, 0, message_len);
	
	lora_tx_ready = 1;

	while (1){
		vTaskDelay(1000 / portTICK_RATE_MS);
		if (lora_tx_ready == 0){
			continue;
		}
		
		message[0] = '#';
		
		if(NEO6_IsFix(&GpsData))
		{
			message[1] = 1;
			float lat = nmea_to_decimal(GpsData.Latitude);
			float lon = nmea_to_decimal(GpsData.Longitude);
			memcpy(message + 2, &lat, 4);
			memcpy(message + 6, &lon, 4);
		}else {
			message[1] = 0;
			memset(message + 2, 0, 8);
		}
		
		if (max_heart_rate != 0) {
			message[10] = 1;
			memcpy(message + 11, &max_heart_rate, 4);
		}else {
			message[10] = 0;
			memset(message + 11, 0, 4);
		}
		
		HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, message, message_len, HAL_MAX_DELAY);
		lora_tx_ready = 0;
		printf("[Lora] Written %d bytes\r\n", message_len);

	}
}

void taskGPSRecv(void *pvParm)
{
	while(1){
		vTaskDelay(10 / portTICK_RATE_MS);
		NEO6_Task(&GpsData);
	}
}

void taskGPSEval(void *pvParm)
{
	while(1){
		vTaskDelay(500 / portTICK_RATE_MS);
		
		if(NEO6_IsFix(&GpsData))
		{
			printf("[GPSModule] Lat: %.4f, Lon: %.4f\r\n", nmea_to_decimal(GpsData.Latitude), nmea_to_decimal(GpsData.Longitude));
		}else {
			printf("[GPSModule] Waiting for the signal...\r\n");
		}
	}
}

void taskMax30102Int(void *pvParm){
	while(1){
		vTaskDelay(10 / portTICK_RATE_MS);
		if (max30102_has_interrupt(&max30102))
    {
				printf("[MAX30102] Has Int\r\n");
				max30102_interrupt_handler(&max30102);
    }
	}
}

void taskMax30102(void* pvParm){
	
	max30102_set_mode(&max30102, max30102_heart_rate);

	printf("[MAX30102] Set Mode\r\n");
	
	int8_t hr_valid = 0;
	int8_t spo2_valid = 0;
	
	int32_t hr_val = 0;
	int32_t spo2_val = 0;
	
	while(1){
		for (int i = 0; i < BUFFER_SIZE; i++)
    {
        max30102_read_fifo(&max30102);
				vTaskDelay(10 / portTICK_RATE_MS);
		}
		
		maxim_heart_rate_and_oxygen_saturation(max30102._ir_samples, 100, max30102._red_samples, &spo2_val, &spo2_valid, &hr_val, &hr_valid); 
		if (hr_valid){
			max_heart_rate = hr_val;
		}
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
} 



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if (GPIO_Pin == GPIO_PIN_15){
			printf("[MAX30102] Int\r\n");
			max30102_on_interrupt(&max30102);
		}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
			  uart_rx_buffer[uart_rx_index] = uart_rx_data;
        uart_rx_index++;
        
        if(uart_rx_data == '\n' || uart_rx_data == '\r')
        {
            uart_rx_buffer[uart_rx_index] = '\0';
            uart_data_ready = 1;
						lora_tx_ready = 1;
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
    }	else if (huart->Instance == USART6){
				NEO6_ReceiveUartChar(&GpsData);
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	printf("[Main] System Init\r\n");
	
	LCD_Init();
	LCD_CS_Clr();
	LCD_Address_Set(0,0,240,240);
	NEO6_Init(&GpsData, &huart6);


	max30102_init(&max30102, &hi2c2);
	max30102_reset(&max30102);
	max30102_clear_fifo(&max30102);
	// FIFO configurations
	max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);
	// LED configurations
  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
  max30102_set_sampling_rate(&max30102, max30102_sr_800);
  max30102_set_led_current_1(&max30102, 6.2);
  max30102_set_led_current_2(&max30102, 6.2);
	
	max30102_set_die_temp_en(&max30102, 1);
  max30102_set_die_temp_rdy(&max30102, 1);
	
	uint8_t en_reg[2] = {0};
  max30102_read(&max30102, 0x00, en_reg, 1);
	
	xTaskCreate(taskGPSRecv, "GPSRecv", 512, NULL, 4, NULL);
	xTaskCreate(taskMax30102Int, "taskMax30102Int", 512, NULL, 4, NULL);
	
	xTaskCreate(taskGPSEval, "GPSEval", 512, NULL, 4, NULL);
	xTaskCreate(taskLora, "taskLora", 1024, NULL, 5, NULL);
	
	xTaskCreate(taskMax30102, "taskMax30102", 512, NULL, 6, NULL);
	
	xTaskCreate(taskST7789, "LCD Display", 256, NULL, 10, NULL);

	printf("[Main] Starting scheduler...\r\n");
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

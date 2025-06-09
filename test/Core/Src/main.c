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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dht11.h"
#include "lcd.h"
#include "lcd_init.h"

#include "tree_1.h"
#include "tree_2.h"
#include "tree_3.h"

#include "stdio.h"
#include "stdbool.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
QueueHandle_t queueDHT11;
QueueHandle_t queueSwitch;
QueueHandle_t queueMode;

TaskHandle_t handleDHT11;
TaskHandle_t handleST7789;
TaskHandle_t handleYL38;
TaskHandle_t handleGPS6MV2;

struct dht11
{
    float Temp;
    float Humi;
};
struct dht11 dht11DATA;

#define UART_BUFFER_SIZE 256
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_data;
volatile uint16_t uart_rx_index = 0;
volatile uint8_t uart_data_ready = 0;

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

/****
DHT11GetData
****/

int env_points = 0;
void taskDHT11(void *pvParm)
{
    while (1)
    {
				printf("In DHT11\r\n");
        if (DHT11GetData(&dht11DATA.Humi, &dht11DATA.Temp) == 0)
        {
           //xQueueSend(queueDHT11, &dht11DATA, NULL);
           printf("Temp:%.1f C", dht11DATA.Temp);
           printf(",Humi:%.1f %%\r\n", dht11DATA.Humi);
        }

				if (dht11DATA.Humi > 60){
					 env_points = env_points + 100;
				} else
				{
					 HAL_GPIO_WritePin(LED_PB14_GPIO_Port, LED_PB14_Pin, 0);
				}

        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}
void taskST7789(void *pvParm){
	uint8_t button = 255;
	uint8_t update = 0;
	int idx = 0;
	uint16_t tick = 0;

	LCD_Fill(0, 0, 240, 240, WHITE);

	while(1){
		//vTaskDelay(1 / portTICK_RATE_MS);
		vTaskDelay(5000 / portTICK_RATE_MS);
		
		printf("current_points: %d\r\n", env_points);
		if (env_points < 500) {
			LCD_ShowPicture(0,0,240,240, gImage_tree_1);
		} 
		else if (env_points < 1000) {
			LCD_ShowPicture(0,0,240,240, gImage_tree_2);
		}
		else if (env_points < 1500){
			LCD_ShowPicture(0,0,240,240, gImage_tree_3);
		}
		else {
			env_points = 0;
		}
	}
}

void taskYL38()
{
	GPIO_PinState do_state;
  char buffer[50];
  
  while(1)
  {
      do_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
      
      sprintf(buffer, "DO state: %s\r\n", 
              do_state == GPIO_PIN_SET ? "dry" : "wet");
      printf("%s\n", buffer);
      
      vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
void taskGPS6MV2()
{
		if(uart_data_ready)
    {
        printf("GPS6MV2: %s\r\n", uart_rx_buffer);
        
        uart_rx_index = 0;
        uart_data_ready = 0;
        memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
    }
}

/****
sw1, 2 -> rising edge
sw3, 4 -> falling edge
****/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t button = 0;

    switch (GPIO_Pin)
    {
    // sw1 PE3
    case GPIO_PIN_3:
    {
        if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1)
        {
					printf("Btn 1 clicked\r\n");
					button = 0;
					xQueueSendFromISR(queueMode, &button, NULL);
        }
        break;
    }
    // sw2 PE4
    case GPIO_PIN_4:
    {
        if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 1)
        {					
					printf("Btn 2 clicked\r\n");
					button = 1;
					xQueueSendFromISR(queueMode, &button, NULL);
        }
        break;
    }
    // sw3 PE5
    case GPIO_PIN_5:
    {
        if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0)
        {
					printf("Btn 3 clicked\r\n");
					button = 2;
					xQueueSendFromISR(queueMode, &button, NULL);
        }
        break;
    }
    default:
        printf("unknown irq \r\n");
        break;
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
        }
        
        HAL_UART_Receive_IT(&huart2, &uart_rx_data, 1);
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
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_CS_Clr();
	LCD_Address_Set(0,0,240,240);
	queueMode = xQueueCreate(3, sizeof(uint8_t));
	xTaskCreate(taskST7789, "LCD Display", 256, NULL, 3, &handleST7789);
	xTaskCreate(taskDHT11, "DHT11", 256, NULL, 4, &handleDHT11);
	xTaskCreate(taskYL38, "YL38", 256, NULL, 4, &handleYL38);
	//xTaskCreate(taskGPS6MV2, "GPS6MV2", 1024, NULL, 4, &handleGPS6MV2);

	printf("Ready to start Scheduler \r\n");
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

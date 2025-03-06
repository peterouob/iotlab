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
#include "ghast.h"
#include "ghast_fire.h"
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
#include "oled.h"
#include "u8g2.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "bongo_cat_1.h"
#include "bongo_cat_2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
QueueHandle_t queueDHT11;
QueueHandle_t queueSwitch;

TaskHandle_t handleOLED;
TaskHandle_t handleDHT11;

u8g2_t u8g2;

struct dht11
{
    float Temp;
    float Humi;
};
struct dht11 dht11DATA;

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
void taskOLED(void *pvParm)
{
    u8g2Init(&u8g2);
    u8g2_ClearBuffer(&u8g2);
    u8g2_ClearDisplay(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_samim_16_t_all);
    uint8_t displayMode = 3;
    uint8_t stuID[9] = "M11317060";
    uint8_t student_id[3][9] = {
        {"B11117020"},
        {"B11117033"},
        {"B11117054"}
		};

    // for (int i = 0; i < 3; ++i)
    //{
    //		int number = ((student_id[i][8] - '0') * 10) + (student_id[i][9] - '0');
    //		if (number % 2)
    //			student_id[i][0] = student_id[i][10] = 'S';
    //		else
    //			student_id[i][0] = student_id[i][10] = 'C';
    //  }
				
		uint8_t student_idx = 0;
		uint8_t student_id_cursor = 0;
		uint8_t animation_tick = 0;

    uint8_t progress = 0;
    char displayTemp[20];
    char displayHumi[20];
    char str_buf[10];
    bool bongo;
		int flag = 0;
		bool init_tmp = false;
		
    while (1)
    {
        xQueueReceive(queueSwitch, &displayMode, NULL);
        BaseType_t dht11Receive = xQueueReceive(queueDHT11, &dht11DATA, NULL);

				switch (displayMode)
				{
					case 0:
					{
							if (dht11Receive == pdPASS)
							{
									snprintf(displayTemp, sizeof(displayTemp), "Temp: %.1f C", dht11DATA.Temp);
									snprintf(displayHumi, sizeof(displayHumi), "Humi: %.1f %%", dht11DATA.Humi);
									u8g2_ClearBuffer(&u8g2);
									u8g2_DrawStr(&u8g2, 0, 20, displayTemp);
									u8g2_DrawStr(&u8g2, 0, 40, displayHumi);
									u8g2_SendBuffer(&u8g2);
									init_tmp = true;
							} else if (!init_tmp){
									u8g2_ClearBuffer(&u8g2);
									u8g2_DrawStr(&u8g2, 0, 20, "Temp: N/A");
									u8g2_DrawStr(&u8g2, 0, 40, "Humi: N/A");
									u8g2_SendBuffer(&u8g2);
							}
							break;
					}
					case 1:
					{
							u8g2_ClearBuffer(&u8g2);
							u8g2_DrawLine(&u8g2, 18, 35, 103, 35);
						
							char last = student_id[student_idx][8];
							if (last % 2)
									u8g2_DrawBox(&u8g2, 5, 20, 10, 10);
							else
									u8g2_DrawCircle(&u8g2, 8, 25, 5, U8G2_DRAW_ALL);
							
							snprintf(str_buf, student_id_cursor < 9 ? student_id_cursor + 2 : 10, "%s", student_id[student_idx]);
							u8g2_DrawStr(&u8g2, 18, 30, str_buf);
							
							if(student_id_cursor >= 9){
								if (last % 2)
										u8g2_DrawBox(&u8g2, 105, 20, 10, 10);
								else
										u8g2_DrawCircle(&u8g2, 108, 25, 5, U8G2_DRAW_ALL);
							}						
							
							u8g2_SendBuffer(&u8g2);
		
						
							student_id_cursor++;
							if(student_id_cursor >= 10){
								vTaskDelay(200);
								student_id_cursor = 0;
								student_idx++;
							}
							
							if(student_idx >= 3){
								student_idx = 0;
							}
							
							vTaskDelay(200);
							break;
					}
					case 2:
					{
							u8g2_ClearBuffer(&u8g2);
							if (bongo)
							{
									u8g2_DrawXBMP(&u8g2, 40, 0, 50, 50, gImage_ghast);
							}
							else
							{
									u8g2_DrawXBMP(&u8g2, 40, 0, 50, 50, gImage_ghast_fire);
							}
							u8g2_SendBuffer(&u8g2);
							vTaskDelay(200);
							break;
					}
					case 3:
					{ // progress bar
							if (progress > 100)
							{
									progress = 0;
							}

							u8g2_ClearBuffer(&u8g2);
							u8g2_DrawBox(&u8g2, 0, 20, progress, 10);
							u8g2_SendBuffer(&u8g2);

							progress += 1;
							vTaskDelay(10);
							break;
					}
					default:
					{
							u8g2_ClearBuffer(&u8g2);
							u8g2_ClearDisplay(&u8g2);
							break;
					}
        }
				
				bongo = !bongo;
    }
}
/****
DHT11GetData
****/
void taskDHT11(void *pvParm)
{
    while (1)
    {
        if (DHT11GetData(&dht11DATA.Humi, &dht11DATA.Temp) == 0)
        {
           xQueueSend(queueDHT11, &dht11DATA, NULL);
           printf("Temp:%.1f C", dht11DATA.Temp);
           printf(",Humi:%.1f %%\r\n", dht11DATA.Humi);
        }

				if (dht11DATA.Humi > 60){
					 HAL_GPIO_WritePin(LED_PB14_GPIO_Port, LED_PB14_Pin, 1);
				} else
				{
					 HAL_GPIO_WritePin(LED_PB14_GPIO_Port, LED_PB14_Pin, 0);
				}

        vTaskDelay(3000);
    }
}

/****
sw1, 2 -> rising edge
sw3, 4 -> falling edge
****/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t mode = 0;

    switch (GPIO_Pin)
    {
    // sw1 PE3
    case GPIO_PIN_3:
    {
        if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1)
        {
            mode = 0;
            printf("display mode:temperature and humidity\r\n");
            xQueueSendFromISR(queueSwitch, &mode, NULL);
        }
        break;
    }
    // sw2 PE4
    case GPIO_PIN_4:
    {
        if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 1)
        {
            mode = 1;
            printf("display mode:student id \r\n");
            xQueueSendFromISR(queueSwitch, &mode, NULL);
        }
        break;
    }
    // sw3 PE5
    case GPIO_PIN_5:
    {
        if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0)
        {
            mode = 2;
            printf("display mode:show picture \r\n");
            xQueueSendFromISR(queueSwitch, &mode, NULL);
        }
        break;
    }
    default:
        mode = 0;
        xQueueSendFromISR(queueSwitch, &mode, NULL);
        printf("unknown irq \r\n");
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
    MX_I2C1_Init();
    MX_USART3_UART_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    queueSwitch = xQueueCreate(3, sizeof(uint8_t));
    queueDHT11 = xQueueCreate(3, sizeof(dht11DATA));

    xTaskCreate(taskDHT11, "DHT11", 128, NULL, 2, &handleDHT11);
    xTaskCreate(taskOLED, "OLED", 512, NULL, 1, &handleOLED);
    printf("StartScheduler \r\n");
    vTaskStartScheduler();
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    if (htim->Instance == TIM1)
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
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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



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
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "tcpclient.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "string.h"
#include "stdio.h"

#include "lcd.h"
#include "lcd_init.h"
#include "image.h"
#include "image_2.h"
#include "image_3.h"
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

QueueHandle_t queueMode;
TaskHandle_t handleST7789;


/*********freeRTOS*********/
TaskHandle_t handleETH;
TaskHandle_t handleTCP_Init;
TaskHandle_t handleTCP_Send;
SemaphoreHandle_t xSemTCP;

/*********LWIP*********/
static struct netconn *connTCP, *newconn;
static struct netbuf *bufTCP;
static ip_addr_t addrDest;
static unsigned short port, portDest;
char msgc[100];
char smsgc[200];
int indx = 0; //send index
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void sendPackage(char *package);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/****************
!!don't remove!!
****************/
FILE __stdout;

int fputc(int c, FILE *f) {
  return (0);
}

/****************
!!don't remove!!
https://reurl.cc/Wx1YDe
****************/

void taskTCP_Init(void *pvParm){
	uint8_t button;
	
	err_t err, accept, recv;
	//create new connection
	connTCP = netconn_new(NETCONN_TCP);
	
	if(connTCP != NULL){
		err = netconn_bind(connTCP, IP_ADDR_ANY, 230);
		if(err == ERR_OK){
			//listen 
			netconn_listen(connTCP);
			//handle connection
			while(1){
				//catch new connection
				accept = netconn_accept(connTCP, &newconn);
				//establish new connection success
				if(accept == ERR_OK){
					//if a client connect into server -> turn on LED_C8
					//HAL_GPIO_WritePin(LED_C8_GPIO_Port, LED_C8_Pin, 1);
					//when server receive data from client -> process buf until end of nuf
					while(netconn_recv(newconn, &bufTCP) == ERR_OK){
						do{
							strncpy(msgc, bufTCP->p->payload, bufTCP->p->len);
							if (*msgc == '0'){
								button = 0;
								xQueueSendFromISR(queueMode, &button, NULL);
							}else if (*msgc == '1'){
								button = 1;
								xQueueSendFromISR(queueMode, &button, NULL);
							}else if (*msgc == '2'){
								button = 2;
								xQueueSendFromISR(queueMode, &button, NULL);
							}
							HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
							memset(msgc, 0, 100);
						}while(netbuf_next(bufTCP) > 0);
						netbuf_delete(bufTCP);
					}
				}
				//if nomore client connect into server -> turn off LED_C8
				//HAL_GPIO_WritePin(LED_C8_GPIO_Port, LED_C8_Pin, 0);
				netconn_close(newconn);
				netconn_delete(newconn);
			}
		}
		//create fail -> delete connection
		else{
			netconn_delete(connTCP);
		}
	}
}

void sendPackage(char *package){
	//send package to connection
	netconn_write(connTCP, package, strlen(package), NETCONN_COPY);
	xSemaphoreGive(xSemTCP);
}

void taskTCP_Send(void *pvParm){
	
	/*
	int idx = 0;
	while(1){
			snprintf(smsgc, sizeof(smsgc), "%d\n", idx++);
			xSemaphoreTake(xSemTCP, 500);
			sendPackage(smsgc);
		vTaskDelay(500);
		}
	*/
	
}



void taskETH(void *pvParm){
	/*************
	init LwIP first
	*************/	
	MX_LWIP_Init();
	
	xSemTCP = xSemaphoreCreateBinary();
	xTaskCreate(taskTCP_Init, "TCP init", 1024, NULL, 0, &handleTCP_Init);
	//xTaskCreate(taskTCP_Send, "TCP send", 1024, NULL, 0, &handleTCP_Send);
	
	while(1){
		HAL_GPIO_TogglePin(LED_C8_GPIO_Port, LED_C8_Pin);
		vTaskDelay(100);
	}
}

void taskST7789(void *pvParm){
	uint8_t button = 255;
	uint8_t update = 0;
	int idx = 0;
	uint16_t tick = 0;

	uint8_t str_student_id[] = "B11117054";
	uint8_t str_hellpigs[] = "Hell Pigs";
	uint8_t str_hellpigs_bj[] = "BJ";
	uint8_t str_hellpigs_gt[] = "Gontone";
	uint8_t str_hellpigs_sun[] = "Sun";

	LCD_Fill(0, 0, 240, 240, WHITE);
	LCD_ShowString(10, 30, str_student_id, BLACK, WHITE, 12, 1);
	LCD_ShowString(10, 60, str_student_id, BLACK, WHITE, 16, 1);
	LCD_ShowString(10, 90, str_student_id, BLACK, WHITE, 24, 1);
	LCD_ShowString(10, 120, str_student_id, BLACK, WHITE, 32, 1);

	while(1){
		vTaskDelay(1 / portTICK_RATE_MS);
		
		xQueueReceive(queueMode, &button, NULL);
		BaseType_t queueModeReceive = xQueueReceive(queueMode, &button, 10 / portTICK_RATE_MS);
		
		if(queueModeReceive == pdPASS){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
			
			update = 1;
			tick = 0;
			
			if (button == 0){
				idx = 0;
				tick = 200;
			} else if (button == 1){
				idx++;
			} else if (button == 2){
				idx--;
			}
		}
		
		if (button == 0){
			if (--tick == 0){
				update = 1;
				idx++;
				tick = 200;
			}
		}
		
		if (!update){
				continue;
		}
		update = 0;
		
		if(idx >= 3){
			idx = 0;
		} else if (idx < 0){
			idx = 2;
		}
		
		
		switch(idx){
			case 0: {
				LCD_ShowPicture(0,0,240,240, gImage_image_2);
				LCD_ShowString(10, 180, str_hellpigs_gt, BLACK, WHITE, 24, 0);
				LCD_ShowString(10, 220, str_hellpigs, BLACK, WHITE, 16, 0);
				break;
			}
			case 1:
			{
				LCD_ShowPicture(0,0,240,240, gImage_image);
				LCD_ShowString(10, 180, str_hellpigs_bj, BLACK, WHITE, 24, 0);
				LCD_ShowString(10, 220, str_hellpigs, BLACK, WHITE, 16, 0);
				break;
			}
			case 2:{
				LCD_ShowPicture(0,0,240,240, gImage_image_3);
				LCD_ShowString(10, 180, str_hellpigs_sun, BLACK, WHITE, 24, 0);
				LCD_ShowString(10, 220, str_hellpigs, BLACK, WHITE, 16, 0);
				break;
			}
			default:
				break;
		}
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_CS_Clr();
	LCD_Address_Set(0,0,240,240);
	queueMode = xQueueCreate(3, sizeof(uint8_t));
	xTaskCreate(taskETH, "ETH example", 1024, NULL, 1, &handleETH);
	xTaskCreate(taskST7789, "LCD Display", 2048, NULL, 5, &handleST7789);
	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
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

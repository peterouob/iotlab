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

QueueHandle_t queueButton;

/*********freeRTOS*********/
TaskHandle_t handleETH;
TaskHandle_t handleTCP_Init;
TaskHandle_t handleTCP_Send;
SemaphoreHandle_t xSemTCP;

/*********LWIP*********/
static struct netconn *connTCP;
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
https://reurl.cc/Wx1YDe`	
****************/

void taskTCP_Init(void *pvParm){
	err_t err, connect;
	//create new connection
	connTCP = netconn_new(NETCONN_TCP);
	
	if(connTCP != NULL){
		
		/******************
		connTCP -> connection which to bind
		IP_ADDR_ANY -> local ip to bind, if use this variable it means to bind all local ip
		7 -> local port to bind
		******************/
		err = netconn_bind(connTCP, IP_ADDR_ANY, 7);
		
		if(err == ERR_OK){
			//server ip, port
			IP_ADDR4(&addrDest, 192, 168, 6, 20);
			portDest = 230;
			
			//connect server
			connect = netconn_connect(connTCP, &addrDest, portDest);
			while (connect != ERR_OK){
				connect = netconn_connect(connTCP, &addrDest, portDest);
				vTaskDelay(50);
			}

				xSemaphoreGive(xSemTCP);
				while(1){
					/*wait for receive the data from server*/
					if(netconn_recv(connTCP, &bufTCP) == ERR_OK){
						/*if rxBuf is not empty, process rxBUF and sent it back*/
						do{
							//take out the data from server sent
							strncpy(msgc, bufTCP->p->payload, bufTCP->p->len);
							//take the data into txBuf
							snprintf(smsgc, sizeof(smsgc), "%s sent from server \n", msgc);
							
							xSemaphoreTake(xSemTCP, 500);
							sendPackage(smsgc);
							
							memset(msgc, 0, 100);
						}while(netbuf_next(bufTCP) > 0);//until all data in rxBuf is processed
						netbuf_delete(bufTCP);
					}
				}
		}
		else{
			//bind fail
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
	while(1){
		uint8_t button = 0;
		uint8_t update = 0;
		
		xQueueReceive(queueButton, &button, NULL);
		BaseType_t queueModeReceive = xQueueReceive(queueButton, &button, 10 / portTICK_RATE_MS);
		
		if(queueModeReceive == pdPASS){
			snprintf(smsgc, sizeof(smsgc), "%d", button);
			xSemaphoreTake(xSemTCP, 500);
			sendPackage(smsgc);
		}
	}
}



void taskETH(void *pvParm){
	/*************
	init LwIP first
	*************/	
	MX_LWIP_Init();
	
	xSemTCP = xSemaphoreCreateBinary();
	xTaskCreate(taskTCP_Init, "TCP init", 1024, NULL, 0, &handleTCP_Init);
	xTaskCreate(taskTCP_Send, "TCP send", 1024, NULL, 0, &handleTCP_Send);
	
	while(1){
		HAL_GPIO_TogglePin(LED_C8_GPIO_Port, LED_C8_Pin);
		vTaskDelay(100);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		uint8_t button = 0;
	
    switch (GPIO_Pin)
    {
    // sw1 PE3
    case GPIO_PIN_3:
    {
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == 1)
        {
					button = 0;
					xQueueSendFromISR(queueButton, &button, NULL);
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        }
        break;
    }
    // sw2 PE4
    case GPIO_PIN_4:
    {
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == 1)
        {
					button = 1;
					xQueueSendFromISR(queueButton, &button, NULL);
        }
        break;
    }
		case GPIO_PIN_5:{
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == 1)
        {
					button = 2;
					xQueueSendFromISR(queueButton, &button, NULL);
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	queueButton = xQueueCreate(3, sizeof(uint8_t));
	xTaskCreate(taskETH, "ETH example", 1024, NULL, 1, &handleETH);
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

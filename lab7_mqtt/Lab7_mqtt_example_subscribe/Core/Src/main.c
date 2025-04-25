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
#include "cmsis_os.h"
#include "i2c.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/apps/mqtt.h"
#include "FreeRTOS.h"
#include "oled.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include "stdbool.h"
#include "task.h"
#include "u8g2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  TOPIC_TEMP = 0,
  TOPIC_HUMI = 1,
  TOPIC_LED = 2,
  TOPIC_UNKNOWN = 3
} TopicId_t;

typedef struct {
  char temp[10];
	char humi[10];
  char ledStatus[10];
} DisplayData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TopicId_t currentTopicId = TOPIC_UNKNOWN;
mqtt_client_t *mqttClient = NULL;

u8g2_t u8g2;
QueueHandle_t queueDisplayData;

SemaphoreHandle_t xSemMqttReconnect;
SemaphoreHandle_t xSemLED;
TaskHandle_t handleMQTT;
TaskHandle_t handleOLED;
TaskHandle_t handleLED;

DisplayData_t displayData = {
  .temp = "0",
	.humi = "0",
  .ledStatus = "Off"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
void mqtt_incoming_topic_cb(void *arg, const char *topic, u32_t tot_len);
void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
void mqtt_sub_request_cb(void *arg, err_t result);

void taskMQTT(void *pvParameters);
void taskOLED(void *pvParameters);
void taskLED(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * MQTT Callback Functions
 */
 
 // Connection callback
void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status){
  if (status == MQTT_CONNECT_ACCEPTED){
    HAL_GPIO_WritePin(LED_B7_GPIO_Port, LED_B7_Pin, GPIO_PIN_SET);
    printf("MQTT connected successfully!\r\n");
  
    mqtt_set_inpub_callback(client, mqtt_incoming_topic_cb, mqtt_incoming_data_cb, NULL);
  
    mqtt_subscribe(client, "Lab7/Temp", 0, NULL, NULL);
    mqtt_subscribe(client, "Lab7/Humi", 0, NULL, NULL);
    mqtt_subscribe(client, "Lab7/LED", 0, NULL, NULL);
  }else{
    HAL_GPIO_WritePin(LED_B7_GPIO_Port, LED_B7_Pin, GPIO_PIN_RESET);
    printf("MQTT connection failed! Error code: %d\r\n", status);
    
    xSemaphoreGive(xSemMqttReconnect);
  }
}

// Incoming topic callback
void mqtt_incoming_topic_cb(void *arg, const char *topic, u32_t tot_len) {
  printf("MQTT: Message received on topic: %s\r\n", topic);

  // Determine topic ID
  if (strcmp(topic, "Lab7/Temp") == 0) {
    currentTopicId = TOPIC_TEMP;
  } else if (strcmp(topic, "Lab7/Humi") == 0) {
    currentTopicId = TOPIC_HUMI;
  } else if (strcmp(topic, "Lab7/LED") == 0) {
    currentTopicId = TOPIC_LED;
  } else{
    currentTopicId = TOPIC_UNKNOWN;
  }
}

// Incoming data callback
void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags){
  // Convert incoming data
  char receiveBuffer[len];
  memcpy(receiveBuffer, data, len);
  receiveBuffer[len] = '\0';
  
  printf("Received message: \"%s\"\r\n", receiveBuffer);
  
  // Data setting
  switch (currentTopicId){
    case TOPIC_TEMP:
      if (len <= sizeof(displayData.temp)){
        memcpy(displayData.temp, receiveBuffer, len);
        displayData.temp[len] = '\0';
      }
      break;
      
    case TOPIC_HUMI:
      if (len <= sizeof(displayData.humi)){
        memcpy(displayData.humi, receiveBuffer, len);
        displayData.humi[len] = '\0';
        
        xSemaphoreGive(xSemLED);
      }
      break;
		case TOPIC_LED:
			if (len <= sizeof(displayData.ledStatus)){
				memcpy(displayData.ledStatus, receiveBuffer, len);
				displayData.ledStatus[len] = '\0';
				
				xSemaphoreGive(xSemLED);
			}
      break;
    default:
      return;
  }
  
  xQueueSend(queueDisplayData, &displayData, NULL);
} 

// Subscribe request callback
void mqtt_sub_request_cb(void *arg, err_t result){
  if (result != ERR_OK){
    printf("Subscribe result: %d\r\n", result);
  }
}

/*
 * FreeRTOS Task Functions
 */

// MQTT initial task
void taskMQTT(void *pvParameters){
  MX_LWIP_Init();
  
  mqttClient = mqtt_client_new();

  struct mqtt_connect_client_info_t mqtt_connect_info = {
    .client_id = "M11317045_B",
    .client_user = "raymiao",
    .client_pass = "548787",
    .keep_alive = 20,
    .will_topic = "Lab7/Last_will",
    .will_msg = "Goodbye client_B!",
    .will_qos = 0,
    .will_retain = 0 
  };
  
	ip4_addr_t ip;
	ip = netif_default->ip_addr;
	printf("STM32 IP: %s\n", ip4addr_ntoa(&ip));

  ip_addr_t mqttBrokerAddr;
	IP_ADDR4(&mqttBrokerAddr, 140, 125, 33, 185);
  err_t connectResult;
  while(1){
    if (!mqtt_client_is_connected(mqttClient)){
      connectResult = mqtt_client_connect(mqttClient, &mqttBrokerAddr, MQTT_PORT, 
                                               mqtt_connection_cb, NULL, &mqtt_connect_info);
      
      if (connectResult != ERR_OK){
        printf("MQTT connect returned error: %d\r\n", connectResult);
        if (connectResult == ERR_ISCONN) {
          mqtt_disconnect(mqttClient);
        }
      }
    } else{
      xSemaphoreTake(xSemMqttReconnect, portMAX_DELAY);
    }
    vTaskDelay(1000);
  }
}

// OLED display task
void taskOLED(void *pvParameters){

  u8g2_ClearBuffer(&u8g2);
  u8g2_ClearDisplay(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_samim_12_t_all);
  
   u8g2_DrawStr(&u8g2, 15, 30, "Waiting...");
   u8g2_DrawStr(&u8g2, 15, 50, "MQTT Connect");
   u8g2_SendBuffer(&u8g2);
   printf("OLED Set okay");
  char displayTemp[20];
	char displayHumi[20];
  char displayLedStatus[20];
  
  while(1){
    xQueueReceive(queueDisplayData, &displayData, portMAX_DELAY);
    
    snprintf(displayTemp, sizeof(displayTemp), "Temp: %s", displayData.temp);
    snprintf(displayHumi, sizeof(displayHumi), "Humi: %s", displayData.humi);
    snprintf(displayLedStatus, sizeof(displayLedStatus), "LED: %s", displayData.ledStatus);
		if (strcmp(displayData.ledStatus, "On") == 0)
		{
			HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, GPIO_PIN_SET);

		}
		else
		{
			HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, GPIO_PIN_RESET);
		}
		
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 15, 30, displayTemp);
		u8g2_DrawStr(&u8g2, 15, 45, displayHumi);
    u8g2_DrawStr(&u8g2, 15, 60, displayLedStatus);
    u8g2_SendBuffer(&u8g2);
  }
}

// LED control task
void taskLED(void *pvParameters){
  HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, GPIO_PIN_RESET);
  
  while(1){
    if (xSemaphoreTake(xSemLED, portMAX_DELAY) == pdTRUE){
      if (strcmp(displayData.ledStatus, "On") == 0) {
        HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, GPIO_PIN_SET);
      } else if (strcmp(displayData.ledStatus, "Off") == 0){
        HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, GPIO_PIN_RESET);
      }
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LED_B14_GPIO_Port, LED_B14_Pin, GPIO_PIN_RESET);

	u8g2Init(&u8g2);
	
  xSemMqttReconnect = xSemaphoreCreateBinary();
  xSemLED = xSemaphoreCreateBinary();
  queueDisplayData = xQueueCreate(3, sizeof(DisplayData_t));
  
  xTaskCreate(taskMQTT, "MQTT", 1024, NULL, 3, &handleMQTT);
  xTaskCreate(taskOLED, "OLED", 256, NULL, 2, &handleOLED);
  xTaskCreate(taskLED, "LED", 256, NULL, 1, &handleLED);
  
   printf("B_StartScheduler \r\n");
   vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

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
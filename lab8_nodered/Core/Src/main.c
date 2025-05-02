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
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include "stdbool.h"
#include "task.h"
#include "dht11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
  TOPIC_SET_HUMI_THRESHOLD = 0,
  TOPIC_SET_LED = 1,
	TOPIC_UNDEFINED = 99
} mqtt_topic_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
mqtt_client_t *mqttClient = NULL;
mqtt_topic_t mqtt_topic = TOPIC_UNDEFINED;


uint8_t humidity_threshold = 50;

QueueHandle_t queueSwitch;

SemaphoreHandle_t xSemMqttReconnect;
TaskHandle_t handleMQTT;
TaskHandle_t handlePublish;
TaskHandle_t handleControl;
TaskHandle_t handleDHT11;
TaskHandle_t handleLED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void mqtt_connection_cb(mqtt_client_t *mqttClient, void *arg, mqtt_connection_status_t status);
void mqtt_pub_request_cb(void *arg, err_t result);
void taskMQTT(void *pvParameters);
void taskPublish(void *pvParameters);
void taskControll(void *pvParameters);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * MQTT Callback Functions
 */
QueueHandle_t queueDHT11;
QueueHandle_t queueSwitch;

struct dht11{
	float Temp;
	float Humi;
};

struct dht11 dht11DATA;


void mqtt_incoming_topic_cb(void *arg, const char *topic, u32_t tot_len) {
  printf("MQTT: Message received on topic: %s\r\n", topic);

  // Determine topic ID
  if (strcmp(topic, "Lab8/Edge/Threshold") == 0) {
    mqtt_topic = TOPIC_SET_HUMI_THRESHOLD;
  } else if (strcmp(topic, "Lab8/Edge/LED") == 0) {
    //mqtt_topic = TOPIC_SET_LED;
		mqtt_topic = TOPIC_UNDEFINED;
	} else{
    mqtt_topic = TOPIC_UNDEFINED;
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
  switch (mqtt_topic){
    case TOPIC_SET_HUMI_THRESHOLD:
			humidity_threshold = (receiveBuffer[0] - '0') * 10 + (receiveBuffer[1] - '0');
      break;
		/*
    case TOPIC_SET_LED:
      if (strcmp(receiveBuffer, "true") == 0){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			}else {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			}
      break;
		*/
    default:
      return;
  }
} 

// Connection status callback
void mqtt_connection_cb(mqtt_client_t *mqttClient, void *arg, mqtt_connection_status_t status){
  if (status == MQTT_CONNECT_ACCEPTED){	
    printf("MQTT connected successfully!\r\n");
    HAL_GPIO_WritePin(LED_B7_GPIO_Port, LED_B7_Pin, GPIO_PIN_SET);
		mqtt_set_inpub_callback(mqttClient, mqtt_incoming_topic_cb, mqtt_incoming_data_cb, NULL);
		mqtt_subscribe(mqttClient, "Lab8/Edge/Threshold", 0, NULL, NULL);
    mqtt_subscribe(mqttClient, "Lab8/Edge/LED", 0, NULL, NULL);
  }else{
    HAL_GPIO_WritePin(LED_B7_GPIO_Port, LED_B7_Pin, GPIO_PIN_RESET);
    printf("MQTT connection failed! Error code: %d\r\n", status);
    
    xSemaphoreGive(xSemMqttReconnect);
  }
}

// Publish request callback
void mqtt_pub_request_cb(void *arg, err_t result){
  if (result != ERR_OK){
    printf("Publish result: %d\r\n", result);
  }
}

/*
 * Task
 */

// MQTT initial task
void taskMQTT(void *pvParameters){
  MX_LWIP_Init();
  
  mqttClient = mqtt_client_new();
  
  struct mqtt_connect_client_info_t mqtt_connect_info = {
    .client_id = "B11117020",
    .client_user = "raymiao",
    .client_pass = "548787",
    .keep_alive = 20,
    .will_topic = "Lab7/Last_will/3",
    .will_msg = "Goodbye client_A!",
    .will_qos = 0,
    .will_retain = 0 
  };

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  static uint8_t mode = 0;

  switch (GPIO_Pin){
    case GPIO_PIN_3:
      if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1){
        mode = 0;
        xQueueSendFromISR(queueSwitch, &mode, NULL);
      }
      break;
  }
}
void taskDHT11(void *pvParm){
	char temp[10];
	char humi[10];
	
  while(1){
    if(DHT11GetData(&dht11DATA.Humi, &dht11DATA.Temp) == 0){
				printf("Get DHT11 Data successfully\n");
        //xQueueSend(queueDHT11, &dht11DATA, NULL);
				snprintf(temp, sizeof(temp), "%.2f", dht11DATA.Temp);
				snprintf(humi, sizeof(humi), "%.2f", dht11DATA.Humi);
				
				mqtt_publish(mqttClient, "Lab8/Dashboard/Temperature", temp, strlen(temp), 0, 0, NULL, NULL);
				mqtt_publish(mqttClient, "Lab8/Dashboard/Humidity", humi, strlen(humi), 0, 0, NULL, NULL);
		}
    vTaskDelay(2000);
	}
}

void taskUpdateLED(void* pvParm){
	char led[4];
	char threshold[3];
	while(1){
		if (dht11DATA.Humi >= humidity_threshold){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			strcpy(led, "On");
			mqtt_publish(mqttClient, "Lab8/Dashboard/LED", led, strlen(led), 0, 0, NULL, NULL);
		}else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			strcpy(led, "Off");
			mqtt_publish(mqttClient, "Lab8/Dashboard/LED", led, strlen(led), 0, 0, NULL, NULL);
		}
		threshold[0] = humidity_threshold / 10 + '0';
		threshold[1] = humidity_threshold % 10 + '0';
		threshold[2] = '\0';
		mqtt_publish(mqttClient, "Lab8/Dashboard/Threshold", threshold, strlen(threshold), 0, 0, NULL, NULL);
    vTaskDelay(1000);
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
  xSemMqttReconnect = xSemaphoreCreateBinary();
  queueSwitch = xQueueCreate(3, sizeof(uint8_t));
	queueDHT11 = xQueueCreate(3, sizeof(dht11DATA));
  xTaskCreate(taskMQTT, "MQTT init", 1024, NULL, 3, &handleMQTT);
	xTaskCreate(taskDHT11, "DHT11", 512, NULL, 2, &handleDHT11);
	xTaskCreate(taskUpdateLED, "LED", 512, NULL, 2, &handleLED);
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

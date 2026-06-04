/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os2.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* Inclusões do FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define CAN_QUEUE_SIZE 128

typedef struct {
    uint32_t id;
    uint8_t data[8];
} CanMessage;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_PRINT    1

#define LORA_SPED      0x3D // UART = 115200bps, LoRa = 19.2kbps
#define LORA_CHANNEL   0x06 // 0x06 = 868MHz, 0x35 = 915MHz
#define LORA_OPTION    0xC4

#define PAYLOAD_SIZE   64
#define TX_INTERVAL_MS 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t txData[PAYLOAD_SIZE] = {
    0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C
};
uint8_t uartRx, i;

/* Manipulador da Fila do FreeRTOS */
static QueueHandle_t canTxQueue = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void disp(const char *format, ...);
void e32_waitAUX(uint32_t timeoutMs);
void e32_setMode(uint8_t m1, uint8_t m0);
uint8_t e32_configureModule(void);
void e32_Send(uint8_t *data, uint32_t canID);

/* Protótipo da Tarefa do FreeRTOS */
void LoRaTxTask(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET);

  disp("FORMULA TESLA UFMG - TELEMETRIA V4 - PLACA TRANSMISSORA\r\n");

  HAL_Delay(200);
  e32_configureModule();
  HAL_Delay(200);

  disp("[MODE] Normal Mode...\r\n");
  e32_setMode(0,0);

  /* 1. Criação da Fila do FreeRTOS */
  canTxQueue = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CanMessage));
  if (canTxQueue == NULL)
  {
      disp("[RTOS] Erro: Falha ao criar a fila de transmissao!\r\n");
      Error_Handler();
  }

  /* 2. Criação da Tarefa do FreeRTOS */
  if (xTaskCreate(LoRaTxTask, "LoRaTX", 256, NULL, 4, NULL) != pdPASS)
  {
      disp("[RTOS] Erro: Falha ao alocar a tarefa LoRaTX!\r\n");
      Error_Handler();
  }

  /* 3. Configuração Segura da Interrupção CAN para o RTOS */
  /* IMPORTANTE: A prioridade deve ser >= 5 para funcionar com chamadas *FromISR */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 5);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
      disp("[CAN] Interrupt Activation Failed!\r\n");
      Error_Handler();
  }
  else
  {
      disp("[CAN] Interrupt Activated!\r\n");
  }
  HAL_FDCAN_Start(&hfdcan1);

  /* 4. Iniciar o Scheduler */
  disp("[RTOS] Iniciando o Scheduler...\r\n");
  vTaskStartScheduler();

  /* O código só chega aqui se faltar Heap para a Idle Task do FreeRTOS */
  disp("[RTOS] FATAL: Memoria Heap insuficiente!\r\n");
  Error_Handler();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Call init function for freertos objects (in app_freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* O while(1) agora fica vazio. O processamento ocorre na LoRaTxTask. */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Tarefa FreeRTOS de Transmissão LoRa.
  * Fica bloqueada (dormindo) até receber mensagens da CAN.
  */
void LoRaTxTask(void *pvParameters)
{
    CanMessage msg;
    for (;;)
    {
        /* Espera indefinidamente (portMAX_DELAY) por um pacote na fila */
        if (xQueueReceive(canTxQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            /* A função e32_Send já gerencia a verificação/espera do pino AUX internamente */
            e32_Send(msg.data, msg.id);
        }
    }
}

// FDCAN Interrupt Handler
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

// FDCAN RX Callback (Modificado para RTOS)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            CanMessage msg;
            msg.id = rxHeader.Identifier;
            memcpy(msg.data, rxData, 8);

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;

            /* Envia a mensagem para a fila e avisa o scheduler se a tarefa for acordada */
            xQueueSendFromISR(canTxQueue, &msg, &xHigherPriorityTaskWoken);

            /* Força a troca de contexto se a tarefa LoRaTx estiver pronta e tiver maior prioridade */
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

            //HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
        }
    }
}

void disp(const char *format, ...)
{
#if DEBUG_PRINT
    char msg[128];
    va_list args;
    va_start(args, format);
    vsnprintf(msg, sizeof(msg), format, args);
    va_end(args);
    HAL_UART_Transmit(
        &huart1,
        (uint8_t*)msg,
        strlen(msg),
        HAL_MAX_DELAY
    );
#endif
}

void e32_waitAUX(uint32_t timeoutMs)
{
    if (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET)
    {
        return;
    }

    uint32_t tickstart = HAL_GetTick();
    while (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - tickstart) > timeoutMs)
        {
            disp("[AUX] Timeout!\r\n");
            return;
        }
    }
    HAL_Delay(2);
}

void e32_setMode(uint8_t m1, uint8_t m0)
{
    HAL_GPIO_WritePin(
        E32_M1_GPIO_Port,
        E32_M1_Pin,
        m1 ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        E32_M0_GPIO_Port,
        E32_M0_Pin,
        m0 ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    HAL_Delay(2);
    e32_waitAUX(2000);
}

uint8_t e32_configureModule(void)
{
    disp("[CFG] Sleep Mode...\r\n");
    e32_setMode(1,1);
    HAL_Delay(100);

    uint8_t dummy;
    while(HAL_UART_Receive(&huart3, &dummy, 1, 10) == HAL_OK);

    uint8_t cfg[6] = {
        0xC0, 0x00, 0x00, LORA_SPED, LORA_CHANNEL, LORA_OPTION
    };

    disp("[CFG] Sending config...\r\n");
    HAL_UART_Transmit(&huart3, cfg, 6, HAL_MAX_DELAY);
    HAL_Delay(300);

    uint8_t resp[6] = {0};
    HAL_UART_Receive(&huart3, resp, 6, 800);

    uint8_t success = 0;
    if(resp[0] == 0xC0)
    {
        disp("[CFG] OK!\r\n");
        success = 1;
    }
    else
    {
        disp("[CFG] Using existing config.\r\n");
        success = 0;
    }

    disp("[CFG] Changing STM32 UART3 Baudrate to 115200...\r\n");
    HAL_UART_DeInit(&huart3);
    huart3.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode(&huart3);

    return success;
}

void e32_Send(uint8_t *data, uint32_t canID)
{
    e32_waitAUX(2000);

    uint8_t packet[3 + 4 + 8];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = LORA_CHANNEL;

    packet[3] = (canID >> 24) & 0xFF;
    packet[4] = (canID >> 16) & 0xFF;
    packet[5] = (canID >> 8)  & 0xFF;
    packet[6] = (canID)       & 0xFF;

    memcpy(&packet[7], data, 8);

    HAL_UART_Transmit(&huart3, packet, 15, HAL_MAX_DELAY);
    HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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

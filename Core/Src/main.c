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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LORA_SPED      0x1C
#define LORA_CHANNEL   0x06
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
uint8_t uartRx;

FDCAN_RxHeaderTypeDef canRxHeader;
uint8_t canRxData[8] = {10, 20, 30, 40, 50, 60, 70, 80};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void e32_waitAUX(uint32_t timeoutMs);
void e32_setMode(uint8_t m1, uint8_t m0);
uint8_t e32_configureModule(void);
void e32_Send(uint8_t *data, uint8_t size, uint32_t canID);

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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET);

  disp("FORMULA TESLA UFMG - TELEMETRIA V4 - PLACA TRANSMISSORA\r\n");


 // HAL_Delay(200);

  e32_configureModule();

  //HAL_Delay(200);

  disp("[MODE] Normal Mode...\r\n");

  e32_setMode(0,0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	e32_Send();

	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
	{
	  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &canRxHeader, canRxData) == HAL_OK)
	  {
		  HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
		  e32_Send(canRxData, 8, canRxHeader.Identifier);
	  }
	}


//	e32_Send(canRxData, 8, 400);
//	HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);

	//HAL_Delay(TX_INTERVAL_MS);
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


void disp(char *msg)
{
    HAL_UART_Transmit(
        &huart1,
        (uint8_t*)msg,
        strlen(msg),
        HAL_MAX_DELAY
    );
}

void e32_waitAUX(uint32_t timeoutMs)
{
    uint32_t tickstart = HAL_GetTick();

    while(HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_RESET)
    {
        if((HAL_GetTick() - tickstart) > timeoutMs)
        {
            //disp("[AUX] Timeout!\r\n");
            return;
        }
    }

    //HAL_Delay(2);
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
        0xC0,
        0x00,
        0x00,
        LORA_SPED,
        LORA_CHANNEL,
        LORA_OPTION
    };

    disp("[CFG] Sending config...\r\n");

    HAL_UART_Transmit(
        &huart3,
        cfg,
        6,
        HAL_MAX_DELAY
    );

    HAL_Delay(300);

    uint8_t resp[6] = {0};

    HAL_UART_Receive(
        &huart3,
        resp,
        6,
        800
    );

    if(resp[0] == 0xC0)
    {
        disp("[CFG] OK!\r\n");
        return 1;
    }
    else
    {
        disp("[CFG] Using existing config.\r\n");
        return 0;
    }

}

void e32_Send(uint8_t *data, uint8_t size, uint32_t canID)
{
    e32_waitAUX(2000);

    /*
        Estrutura do pacote:

        [0]  = 0xFF
        [1]  = 0xFF
        [2]  = canal LoRa

        [3]  = CAN ID byte 3
        [4]  = CAN ID byte 2
        [5]  = CAN ID byte 1
        [6]  = CAN ID byte 0

        [7]  = tamanho payload CAN

        [8...] = dados CAN
    */

    uint8_t packet[3 + 4 + 8];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = LORA_CHANNEL;

    // CAN ID
    packet[3] = (canID >> 24) & 0xFF;
    packet[4] = (canID >> 16) & 0xFF;
    packet[5] = (canID >> 8)  & 0xFF;
    packet[6] = (canID)       & 0xFF;


    // payload CAN
    memcpy(&packet[7], data, size);

    HAL_UART_Transmit(
        &huart3,
        packet,
        7 + size,
        HAL_MAX_DELAY
    );

    //printf("[LORA TX] CAN ID: 0x%lX | %d bytes\r\n", canID, size);

    e32_waitAUX(2000);
}
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

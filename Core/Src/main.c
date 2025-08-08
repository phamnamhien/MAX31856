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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max31856.h"
#include <stdio.h>
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
MAX31856_Handle_t tc_sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// Platform functions for MAX31856
max31856_status_t platform_spi_write_read(MAX31856_Handle_t *hmax, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
void platform_cs_low(void);
void platform_cs_high(void);
void platform_delay_ms(uint32_t ms);
int _write(int file, char *ptr, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief SPI communication for MAX31856
 */
max31856_status_t platform_spi_write_read(MAX31856_Handle_t *hmax, uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    if (HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size, 1000) == HAL_OK) {
        return MAX31856_OK;
    }
    return MAX31856_TIMEOUT;
}

/**
 * @brief Set CS pin low
 */
void platform_cs_low(void) {
    HAL_GPIO_WritePin(MAX31856_CS_GPIO_Port, MAX31856_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Set CS pin high
 */
void platform_cs_high(void) {
    HAL_GPIO_WritePin(MAX31856_CS_GPIO_Port, MAX31856_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Delay function
 */
void platform_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

/**
 * @brief Redirect printf to UART
 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);
    return len;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n=== MAX31856 Simple Example ===\r\n");

   // Create platform interface
   max31856_platform_t platform = {
       .spi_write_read = platform_spi_write_read,
       .cs_low = platform_cs_low,
       .cs_high = platform_cs_high,
       .delay_ms = platform_delay_ms,
       .platform_data = NULL
   };

   // Initialize MAX31856 with Type K thermocouple
   if (MAX31856_Init(&tc_sensor, &platform, NULL, 0, MAX31856_TC_TYPE_K) == MAX31856_OK) {
       printf("MAX31856 initialized successfully!\r\n");
   } else {
       printf("MAX31856 initialization failed!\r\n");
       while(1); // Stop here if init fails
   }

   // Enable open-circuit detection
   MAX31856_SetOCFault(&tc_sensor, MAX31856_OCFAULT_10MS);
   printf("Ready to measure temperature...\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	float tc_temp, cj_temp;
	uint8_t fault;

	// Read thermocouple temperature
	if (MAX31856_ReadTemperature(&tc_sensor, &tc_temp) == MAX31856_OK) {
		printf("TC: %.2f C", tc_temp);
	} else {
		printf("TC: Error");
	}

	// Read cold-junction temperature
	if (MAX31856_ReadCJTemperature(&tc_sensor, &cj_temp) == MAX31856_OK) {
		printf(", CJ: %.2f C", cj_temp);
	} else {
		printf(", CJ: Error");
	}

	// Check for faults
	if (MAX31856_ReadFault(&tc_sensor, &fault) == MAX31856_OK && fault != 0) {
		printf(", Fault: 0x%02X", fault);
	if (fault & MAX31856_FAULT_OPEN) {
		printf(" (OPEN!)");
	}
		MAX31856_ClearFault(&tc_sensor);
	}

	printf("\r\n");

	HAL_Delay(2000); // Update every 2 seconds
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "usbd_cdc_if.h"
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
//**************************ADS1115 variables ///
const uint8_t ads1115_address_w = 0b01001000 << 1;
const uint8_t ads1115_address_r = 0b01001000 << 1 | 0x01;
const uint8_t ads1115_reg_config = 0b00000001;
uint8_t ads1115_config_data[] = { 0b10001110, 0b11100011 };
const uint8_t ads1115_reg_conversion = 0b00000000;

uint8_t adc_value[2];
int16_t converted;
char buffer[25];
char duration[5];
long dur_number;
char *ptr;
//**************************ADS1115 variables ///

uint8_t DataToSend[40]; // Tablica zawierajaca dane do wyslania
uint8_t MessageCounter = 0; // Licznik wyslanych wiadomosci
uint8_t MessageLength = 0; // Zawiera dlugosc wysylanej wiadomosci

uint8_t ReceivedData[40]; // Tablica przechowujaca odebrane dane
uint8_t ReceivedDataFlag = 0; // Flaga informujaca o odebraniu danych
const char resetAll[] = "RESET";
const char startADC[] = "ADC";
const char a0on[] = "A0 ON";
const char a1on[] = "A1 ON";
const char a2on[] = "A2 ON";
const char a3on[] = "A3 ON";
const char a0off[] = "A0 OFF";
const char a1off[] = "A1 OFF";
const char a2off[] = "A2 OFF";
const char a3off[] = "A3 OFF";
const char b5[] = "B5?";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	HAL_I2C_Mem_Write(&hi2c1, ads1115_address_w, ads1115_reg_config, 1,
			ads1115_config_data, 2, 50);
	HAL_I2C_Master_Transmit(&hi2c1, ads1115_address_w, ads1115_reg_conversion,
			1, 50);

	HAL_GPIO_WritePin(PB4_ALWAYS_HIGH_GPIO_Port, PB4_ALWAYS_HIGH_Pin,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (ReceivedDataFlag == 1) {
			ReceivedDataFlag = 0;
			// odczyt ADS1115//
			if (strncmp((const char*) ReceivedData, startADC, 3) == 0) {
				for (int var = 0; var < 5; ++var) {
					duration[var] = ReceivedData[3 + var];
				}
				dur_number = strtol(duration, &ptr, 10);
				for (int var = 0; var < dur_number; ++var) {
					HAL_Delay(5);
					HAL_I2C_Master_Receive(&hi2c1, ads1115_address_r, adc_value,
							2, 50);
					converted = ((adc_value[0] << 8) | adc_value[1]);
					snprintf(buffer, sizeof(buffer) - 1, "%d\n", converted);
					CDC_Transmit_FS(buffer, sizeof(buffer));
				}
			}
			// sterowanie przekaznikami
			if (strncmp((const char*) ReceivedData, resetAll, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
				MessageLength = sprintf(DataToSend,
						"Zresetowano wszystkie przekazniki\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a0on, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				MessageLength = sprintf(DataToSend, "Wlaczono A0\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a1on, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
				MessageLength = sprintf(DataToSend, "Wlaczono A1\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a2on, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
				MessageLength = sprintf(DataToSend, "Wlaczono A2\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a3on, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				MessageLength = sprintf(DataToSend, "Wlaczono A3\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a0off, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				MessageLength = sprintf(DataToSend, "Wylaczono A0\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a1off, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
				MessageLength = sprintf(DataToSend, "Wylaczono A1\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a2off, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				MessageLength = sprintf(DataToSend, "Wylaczono A2\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}
			if (strncmp((const char*) ReceivedData, a3off, 5) == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
				MessageLength = sprintf(DataToSend, "Wylaczono A3\n");
				CDC_Transmit_FS(DataToSend, MessageLength);
			}

			// odczyt stanu pinu
			if (strncmp((const char*) ReceivedData, b5, 3) == 0) {
				if (HAL_GPIO_ReadPin(PB5_GPIO_Port, PB5_Pin) == GPIO_PIN_SET) {
					MessageLength = sprintf(DataToSend, "1\n");
					CDC_Transmit_FS(DataToSend, MessageLength);
				} else {
					MessageLength = sprintf(DataToSend, "0\n");
					CDC_Transmit_FS(DataToSend, MessageLength);
				}
			}

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
#include "string.h"
#include <stdlib.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "servo.h"
#include "vl53l0x.h"
#include "pid.h"
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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

Servo_Handle_t servo;
VL53L0X_Dev_t tof;
uint16_t distance;
char msg[64];

// Zmienne do odbioru UART
uint8_t rx_byte;
uint8_t rx_buffer[64];
volatile uint8_t rx_idx = 0;
volatile uint8_t cmd_received = 0;

// Zmienne Globalne Sterowania
volatile float g_setpoint = 200.0f;
volatile float g_Kp = -0.5f;
volatile float g_Ki = -0.0004f;
volatile float g_Kd = -250.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// --- Adaptive EMA Struct ---
typedef struct {
	float min_alpha;
	float max_alpha;
	float threshold;
	float value;
	uint8_t first_run;
} AdaptiveEMA_t;

void AdaptiveEMA_Init(AdaptiveEMA_t *ema, float min_alpha, float max_alpha, float threshold) {
	ema->min_alpha = min_alpha;
	ema->max_alpha = max_alpha;
	ema->threshold = threshold;
	ema->value = 0.0f;
	ema->first_run = 1;
}

float AdaptiveEMA_Filter(AdaptiveEMA_t *ema, float measurement) {
	if (ema->first_run) {
		ema->value = measurement;
		ema->first_run = 0;
		return measurement;
	}

	float error = (measurement > ema->value) ? (measurement - ema->value) : (ema->value - measurement); // abs(float)

	// Factor = min(error / threshold, 1.0)
	float factor = error / ema->threshold;
	if (factor > 1.0f)
		factor = 1.0f;

	float alpha = ema->min_alpha + (ema->max_alpha - ema->min_alpha) * factor;

	ema->value = alpha * measurement + (1.0f - alpha) * ema->value;
	return ema->value;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t CalculateCRC8(const char *data, int len) {
	uint8_t crc = 0x00;
	for (int i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
		}
	}
	return crc;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	// Dla serw 180/200 stopni (jak MG946R) zakres impulsu to zazwyczaj 500us - 2500us
	// 500us = 0 stopni, 2500us = max (ok. 180-200 stopni)
	const uint32_t MIN_CCR = 500;
	const uint32_t MAX_CCR = 2500;
	const uint16_t MAX_ANGLE = 200;

	Servo_Init(&servo, &htim3, TIM_CHANNEL_1, MIN_CCR, MAX_CCR, MAX_ANGLE);

	// I2C Scanner
	HAL_UART_Transmit(&huart3, (uint8_t*) "Scanning I2C bus...\r\n", 21, 100);
	int devices_found = 0;
	for (uint16_t i = 1; i < 128; i++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 1, 10) == HAL_OK) {
			char scanMsg[32];
			sprintf(scanMsg, "Found device at 0x%02X\r\n", (i << 1));
			HAL_UART_Transmit(&huart3, (uint8_t*) scanMsg, strlen(scanMsg), 100);
			devices_found++;
		}
	}
	if (devices_found == 0) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "No I2C devices found!\r\n", 23, 100);
	}
	HAL_UART_Transmit(&huart3, (uint8_t*) "Scan complete.\r\n", 16, 100);

//	Servo_Start(&servo);
//	Servo_SetAngle(&servo, 100); // Start od 0

	// Manual DEBUG: Read Model ID explicitly
	uint8_t id_val = 0;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0x52, 0xC0, I2C_MEMADD_SIZE_8BIT, &id_val, 1, 100);

	char debugMsg[64];
	if (status == HAL_OK) {
		sprintf(debugMsg, "Read Model ID: 0x%02X (Expected 0xEE)\r\n", id_val);
	} else {
		sprintf(debugMsg, "I2C Read Error: %d\r\n", status);
	}
	HAL_UART_Transmit(&huart3, (uint8_t*) debugMsg, strlen(debugMsg), 100);

	// Init VL53L0X
	if (VL53L0X_Init(&tof, &hi2c1)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Init OK\r\n", 17, 100);
		// USUWAMY STARY OFFSET (-40)
		// VL53L0X_SetOffset(&tof, -40);
	} else {
		HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Init FAILED\r\n", 21, 100);
	}

  /* USER CODE END 2 */
  
  // === INICJALIZACJA PID ===
	PID_Controller_t pid;

	// Sugerowane wartości startowe
	g_Kp = -0.11f;
	g_Ki = -0.004f;
	g_Kd = -2.0f;

	// Inicjalizacja PID
	PID_Init(&pid, g_Kp, g_Ki, g_Kd, -35.0f, 35.0f);

	// Zmienne do średniej kroczącej
#define MOVING_AVG_SIZE 5
	float dist_history[MOVING_AVG_SIZE] = { 0 };
	int dist_idx = 0;
	float dist_sum = 0;

	// Zmienna czasu
	uint32_t last_pid_time = 0;

	Servo_Start(&servo);
	// Ustawienie początkowe na środek (zakładamy 100 jako środek geometryczny belki)
	float current_servo_angle = 100.0f;
	Servo_SetAngle(&servo, (uint16_t) current_servo_angle);


   HAL_UART_Transmit(&huart3, (uint8_t*) "Entering main loop...\r\n", 23, 100);
   HAL_UART_Receive_IT(&huart3, &rx_byte, 1); // Start Rx

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    		// --- PARSOWANIE KOMEND ---
		if (cmd_received) {
			cmd_received = 0;
            
            // DEBUG ECHO: Sprawdzamy co STM32 widzi w buforze
            char debug_buf[80];
            sprintf(debug_buf, "LOG:RX_RAW=[%s]\r\n", rx_buffer);
            HAL_UART_Transmit(&huart3, (uint8_t*)debug_buf, strlen(debug_buf), 100);

			// Format: CMD...;C:CRC
			char *crc_ptr = strstr((char*) rx_buffer, ";C:");
			if (crc_ptr) {
				*crc_ptr = 0; // Null term data part
				char *crc_hex = crc_ptr + 3;
				uint8_t rec_crc = (uint8_t) strtol(crc_hex, NULL, 16);
				uint8_t calc_crc = CalculateCRC8((char*) rx_buffer, strlen((char*) rx_buffer));

				if (rec_crc == calc_crc) {
					// Parsowanie
					if (strncmp((char*) rx_buffer, "SET:", 4) == 0) {
						float val = (float)atof((char*) rx_buffer + 4);
						if (val >= 0 && val <= 300) {
							g_setpoint = val;
							// Potwierdzenie ACK
							char ack_msg[64];
							sprintf(ack_msg, "LOG:ACK SET:%d\r\n", (int)g_setpoint);
							HAL_UART_Transmit(&huart3, (uint8_t*) ack_msg, strlen(ack_msg), 100);
						}
					} else if (strncmp((char*) rx_buffer, "PID:", 4) == 0) {
						// PID:Kp;Ki;Kd
						// Parsowanie ręczne (strtok-style) dla pewności
						char* p_ptr = (char*)rx_buffer + 4;
						char* i_ptr = strchr(p_ptr, ';');
						
						if (i_ptr) {
							*i_ptr = '\0'; // Zakończ string P
							i_ptr++;      // Przesuń na I
							
							char* d_ptr = strchr(i_ptr, ';');
							if (d_ptr) {
								*d_ptr = '\0'; // Zakończ string I
								d_ptr++;      // Przesuń na D
								
								// Używamy atof (stdlib.h required)
								g_Kp = (float)atof(p_ptr); 
								g_Ki = (float)atof(i_ptr);
								g_Kd = (float)atof(d_ptr);

								// Re-init PID
								PID_Reset(&pid);
								PID_Init(&pid, g_Kp, g_Ki, g_Kd, -35.0f, 35.0f);

								// LOG ACK - formatowanie x1000/x100000 dla czytelności
								char ack_msg[64];
								sprintf(ack_msg, "LOG:ACK P:%d I:%d D:%d\r\n", 
										(int)(g_Kp*1000), (int)(g_Ki*100000), (int)(g_Kd*10));
								HAL_UART_Transmit(&huart3, (uint8_t*) ack_msg, strlen(ack_msg), 100);
							}
						}
					}
				} else {
					HAL_UART_Transmit(&huart3, (uint8_t*)"LOG:CRC FAIL\r\n", 14, 100);
				}
			}
		}

		// 1. CZYTANIE CZUJNIKA (Non-stop)
		distance = VL53L0X_ReadDistance(&tof);

		if (distance != 0xFFFF && distance < 8190) {
			// Korekta na promień kulki (średnica 40mm -> promień 20mm)
			// Czujnik widzi powierzchnię, my chcemy środek.
			distance += 20;

			// Średnia krocząca
			dist_history[dist_idx] = (float) distance;
			dist_idx = (dist_idx + 1) % MOVING_AVG_SIZE;

			// Policz średnią
			dist_sum = 0;
			for (int i = 0; i < MOVING_AVG_SIZE; i++)
				dist_sum += dist_history[i];
			float avg_dist = dist_sum / (float) MOVING_AVG_SIZE;

			// 2. OBLICZANIE PID (Tylko co 30ms)
			if (HAL_GetTick() - last_pid_time >= 30) {
				last_pid_time = HAL_GetTick();

				// Używamy uśrednionej wartości avg_dist
				float pid_out = PID_Compute(&pid, g_setpoint, avg_dist);

				float target_servo_angle = 100.0f + pid_out;
				float alpha = 0.4f; // Zachowujemy wygładzanie wyjścia serwa
				current_servo_angle = current_servo_angle * (1.0f - alpha) + target_servo_angle * alpha;

				if (current_servo_angle > 135.0f)
					current_servo_angle = 135.0f;
				if (current_servo_angle < 65.0f)
					current_servo_angle = 65.0f;

				Servo_SetAngle(&servo, (uint16_t) current_servo_angle);

				// === WYSYŁANIE Z CRC ===
				float current_error = g_setpoint - avg_dist;
				char data_buffer[64];
				int len = sprintf(data_buffer, "D:%d;A:%d;F:%d;E:%d", distance, (int) current_servo_angle,
						(int) avg_dist, (int) current_error);
				uint8_t out_crc = CalculateCRC8(data_buffer, len);

				sprintf(msg, "%s;C:%02X\r\n", data_buffer, out_crc);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
			}

		} else {
			// Opcjonalnie log bledow rzadziej
		}
		// Brak HAL_Delay - max speed loop
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
        // Toggle LED to indicate activity
        HAL_GPIO_TogglePin(GPIOB, LD1_Pin); 

		if (rx_byte == '\n' || rx_byte == '\r') {
			// Koniec ramki
			if (rx_idx > 0) {
				rx_buffer[rx_idx] = 0; // Null terminate
				cmd_received = 1;
			}
			rx_idx = 0; // Reset bufora
		} else {
			if (rx_idx < 63) {
				rx_buffer[rx_idx++] = rx_byte;
			} else {
				// Overflow protection
				rx_idx = 0;
			}
		}
		// Wznów nasłuchiwanie
		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		// ORE (Overrun) or Noise Error. Restart.
        // Opcjonalnie: Toggle Red LED
        HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	}
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
	while (1) {
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
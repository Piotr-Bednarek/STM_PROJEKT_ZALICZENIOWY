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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "VL53L0X.h"
#include <stdio.h>
#include <stdlib.h>
#include "filters.h"
#include "calibration.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// --- Parametry PID ---
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float prevError;
	float integral;
	float prevMeasurement;  // Do obliczania pochodnej na podstawie pomiaru (Derivative-on-Measurement)
	EMA_Filter_t d_filter;  // Filtr dolnoprzepustowy dla członu różniczkującego
} ServoPID_Controller;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Konfiguracja rampy
#define SETPOINT_DEFAULT   100.0f
#define BALL_RADIUS        20
#define PID_DT_MS          30

// Zakres ruchu serwa
#define SERVO_CENTER       100.0f
#define SERVO_MIN_LIMIT    50.0f
#define SERVO_MAX_LIMIT    150.0f

// Ustawienia sprzętowe Serwa (PWM)
#define SERVO_MIN_CCR      500
#define SERVO_MAX_CCR      2500
#define SERVO_MAX_ANGLE    200

// Tryby
#define SENSOR_TEST_MODE   0  // 1: logowanie surowych danych (debugowanie)
#define USE_CALIBRATION    1  // 1: użyj kalibracji 5-punktowej, 0: surowe dane

// Ustawienia PID i Filtracji
#define D_DEADBAND         0.5f   // mm - strefa nieczułości dla członu D
#define D_FILTER_ALPHA     0.25f  // Wygładzanie dla członu D (0.0 - 1.0)

// Wygładzanie Serwa
#define SERVO_ANGLE_DEADBAND  0.0f // Strefa nieczułości serwa (0.8 stopnia) - ignoruje małe drgania
#define SERVO_SMOOTHING_SIZE  1    // Ilość próbek do wygładzania ruchu

#define AVG_ERR_SAMPLES       35   // Liczba próbek do średniego uchybu (ok. 1 sekunda)
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

ADC_HandleTypeDef hadc1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId ControlTaskHandle;
/* USER CODE BEGIN PV */
uint16_t distance;
char msg[64];

// Zmienne obsługi UART
uint8_t rx_byte;
uint8_t rx_buffer[64];
volatile uint8_t rx_idx = 0;
volatile uint8_t cmd_received = 0;

// Główne zmienne sterujące
// Pozycja piłki: 0 mm (start) - 290 mm (koniec)
volatile float g_setpoint = 145.0f; // Domyślnie środek belki
volatile float g_Kp = 0.15f;       // Wzmocnienie proporcjonalne (dodatnie po obrocie serwa)
volatile float g_Ki = 0.0025f;     // Wzmocnienie całkujące (dodatnie po obrocie serwa)
volatile float g_Kd = 6.0f;        // Wzmocnienie różniczkujące (dodatnie po obrocie serwa)

// Zmienne kalibracji
volatile uint8_t calibration_mode = 0;
volatile float cal_raw_min = 9999.0f;
volatile float cal_raw_max = 0.0f;
volatile uint32_t cal_start_time = 0;

// Domyślne punkty kalibracyjne
volatile float sensor_min = 50.0f;
volatile float sensor_max = 220.0f;
volatile float sensor_middle = 115.0f;

// Bufor średniego uchybu
float err_buffer[AVG_ERR_SAMPLES];
uint8_t err_idx = 0;
float err_sum = 0.0f;

// Tryb sterowania: 0 = GUI (Manual), 1 = Analog (Potencjometr)
volatile uint8_t control_mode = 0;

// Debugging
volatile uint32_t g_adc_raw = 0;
volatile float g_pot_setpoint = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);

/* USER CODE BEGIN PFP */

void SetServoAngle(float angle);
void ServoPID_Init(ServoPID_Controller *pid);
float ServoPID_Compute(ServoPID_Controller *pid, float error, float measurement);
void TestLED(); // Funkcja testowa dla weryfikacji połączenia LED

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (cmd_received == 0) {
			if (rx_byte == '\n' || rx_byte == '\r') {
				rx_buffer[rx_idx] = '\0';
				if (rx_idx > 0)
					cmd_received = 1;
				rx_idx = 0;
			} else {
				if (rx_idx < 63) {
					rx_buffer[rx_idx++] = rx_byte;
				}
			}
		}
		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	}
}

void ServoPID_Init(ServoPID_Controller *pid) {
	pid->Kp = 0.0f;
	pid->Ki = 0.0f;
	pid->Kd = 0.0f;
	pid->prevError = 0.0f;
	pid->integral = 0.0f;
	pid->prevMeasurement = 0.0f;
	EMA_Init(&pid->d_filter, D_FILTER_ALPHA);
}

float ServoPID_Compute(ServoPID_Controller *pid, float error, float measurement) {
	pid->Kp = g_Kp;
	pid->Ki = g_Ki;
	pid->Kd = g_Kd;

	// 1. Obliczamy D (Derivative) najpierw, aby znać jej wpływ na wyjście
	float D = 0.0f;
	float raw_derivative = -(measurement - pid->prevMeasurement);

	if (raw_derivative > -D_DEADBAND && raw_derivative < D_DEADBAND) {
		raw_derivative = 0.0f;
	}
	float filtered_derivative = EMA_Update(&pid->d_filter, raw_derivative);
	D = pid->Kd * filtered_derivative;
	pid->prevMeasurement = measurement; // Update state

	// 2. Obliczamy P (Proportional)
	float P = pid->Kp * error;

	// 3. Obliczamy Anti-Windup (Clamping)
	// Obliczamy wyjście bez nowej całki (używamy starej całki)
	float old_I = pid->Ki * pid->integral;
	float tentative_output = SERVO_CENTER + P + old_I + D;

	int saturated = 0;
	// Sprawdzamy czy to wyjście przekracza limity
	if (tentative_output > SERVO_MAX_LIMIT) {
		saturated = 1; // Nasycenie górne
	} else if (tentative_output < SERVO_MIN_LIMIT) {
		saturated = -1; // Nasycenie dolne
	}

	// Decyzja o całkowaniu:
	// Całkujemy jeśli:
	// a) Nie ma nasycenia (saturated == 0)
	// b) Nasycenie jest górne (1), ale błąd * Ki ma znak ujemny (chce zmniejszyć wyjście)
	// c) Nasycenie jest dolne (-1), ale błąd * Ki ma znak dodatni (chce zwiększyć wyjście)

	// Uwaga: Znak zmiany całki zależy od znaku (error * Ki).
	// Jeśli Ki jest ujemne, to error>0 zmniejsza całkę (ujemny wkład).

	float integration_contribution = error * pid->Ki;

	if (saturated == 0) {
		pid->integral += error;
	} else if (saturated == 1 && integration_contribution < 0) {
		// Jesteśmy na MAX limicie, ale sterownik chce zmniejszyć wyjście -> POZWÓL CAŁKOWAĆ
		pid->integral += error;
	} else if (saturated == -1 && integration_contribution > 0) {
		// Jesteśmy na MIN limicie, ale sterownik chce zwiększyć wyjście -> POZWÓL CAŁKOWAĆ
		pid->integral += error;
	}

	// Twardy limit całki (failsafe)
	if (pid->integral > 3000.0f)
		pid->integral = 3000.0f;
	if (pid->integral < -3000.0f)
		pid->integral = -3000.0f;

	// 4. Finalne wyjście
	float new_I = pid->Ki * pid->integral;
	float output = SERVO_CENTER + P + new_I + D;

	pid->prevError = error;

	return output;
}

void SetServoAngle(float angle) {
	// Sterowanie serwem SG90/MG996R:
	// 0 stopni = impuls ~500us
	// 180 stopni  = impuls ~2500us

	if (angle < 0.0f)
		angle = 0.0f;
	if (angle > 180.0f)
		angle = 180.0f;

	uint32_t pulse_length = (uint32_t) (500.0f + (angle / 180.0f) * 2000.0f);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_length);
}

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

// Funkcja testowa - zapala/gasi LED_ERR_1 (PE2) co 500ms
void TestLED() {
	static uint32_t last_toggle = 0;
	
	if (HAL_GetTick() - last_toggle > 500) {
		HAL_GPIO_TogglePin(LED_ERR_PORT, LED_ERR_1_Pin);
		last_toggle = HAL_GetTick();
	}
}

// Funkcja docelowa dla 5 diod (R-Ż-Z-Ż-R)
void UpdateErrorLEDs_5LED(float error) {
	// Wyłącz wszystkie LEDy (PE2..PE6)
	HAL_GPIO_WritePin(GPIOE, LED_ERR_1_Pin | LED_ERR_2_Pin | LED_ERR_3_Pin | 
	                         LED_ERR_4_Pin | LED_ERR_5_Pin, GPIO_PIN_RESET);
	
	// Progi: 
	// 1% z 290mm = 2.9mm
	// 5% z 290mm = 14.5mm
	float limit_green = 2.9f;
	float limit_yellow = 14.5f;
	
	if (error < -limit_yellow) {
		// Duży ujemny -> Czerwona (PE2 - LED_ERR_1)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_1_Pin, GPIO_PIN_SET);
	} else if (error < -limit_green) {
		// Średni ujemny -> Żółta (PE3 - LED_ERR_2)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_2_Pin, GPIO_PIN_SET);
	} else if (error <= limit_green) { 
		// Mały błąd (+/- 2.9mm) -> Zielona (PE4 - LED_ERR_3)
		// Uwaga: W main.h zdefiniowalismy LED_ERR_3_Pin jako PE4
		HAL_GPIO_WritePin(GPIOE, LED_ERR_3_Pin, GPIO_PIN_SET);
	} else if (error <= limit_yellow) {
		// Średni dodatni -> Żółta (PE5 - LED_ERR_4)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_4_Pin, GPIO_PIN_SET);
	} else {
		// Duży dodatni -> Czerwona (PE6 - LED_ERR_5)
		HAL_GPIO_WritePin(GPIOE, LED_ERR_5_Pin, GPIO_PIN_SET);
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
	HAL_Delay(500); // Czekaj na ustabilizowanie zasilania czujników
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	// Initialization moved to StartControlTask
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_ERR_1_Pin|LED_ERR_2_Pin|LED_ERR_3_Pin|LED_ERR_4_Pin
                          |LED_ERR_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_ERR_1_Pin LED_ERR_2_Pin LED_ERR_3_Pin LED_ERR_4_Pin
                           LED_ERR_5_Pin */
  GPIO_InitStruct.Pin = LED_ERR_1_Pin|LED_ERR_2_Pin|LED_ERR_3_Pin|LED_ERR_4_Pin
                          |LED_ERR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

  /*Configure GPIO pins : CALIB_START_BTN_Pin CALIB_MID_BTN_Pin CALIB_END_BTN_Pin */
  GPIO_InitStruct.Pin = CALIB_START_BTN_Pin|CALIB_MID_BTN_Pin|CALIB_END_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief Function implementing the ControlTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
	// Uruchomienie PWM dla serwa (Timer 3, Kanał 1)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	// Czekamy 2 sekundy, aby kondensatory się naładowały i napięcie ustabilizowało.
	HAL_UART_Transmit(&huart3, (uint8_t*) "Stabilizing Power...\r\n", 22, 100);
	osDelay(2000);

	HAL_UART_Transmit(&huart3, (uint8_t*) "Setting Servo to CENTER\r\n", 25, 100);
	SetServoAngle(SERVO_CENTER); // Ustaw serwo na środek (100 stopni)

	// Test led
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	osDelay(200);
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);

	// Inicjalizacja czujnika odległości VL53L0X
	statInfo_t_VL53L0X distanceStr;
	osDelay(100);

	HAL_UART_Transmit(&huart3, (uint8_t*) "Resetting VL53L0X...\r\n", 21, 100);

	uint8_t reset_val = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, ADDRESS_DEFAULT, 0x00BF, 1, &reset_val, 1, 100);
	osDelay(50); // Czas na reset

	HAL_UART_Transmit(&huart3, (uint8_t*) "Initializing VL53L0X...\r\n", 25, 100);
	if (!initVL53L0X(1, &hi2c1)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Init Failed!\r\n", 22, 100);
	} else {
		HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Init Success!\r\n", 23, 100);
	}

	startContinuous(0);

	HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Ready! Sending data...\r\n", 32, 100);

	Calibration_Init();

	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

	uint32_t loop_counter = 0;

	// Inicjalizacja filtrów
	OneEuroFilter_t one_euro;
	OneEuro_Init(&one_euro, 2.0f, 0.001f, 1.0f);

	MedianFilter_t median_filter;
	MedianFilter_Init(&median_filter);

	// Lekki filtr EMA dla czujnika (Alpha mniejsze = mocniejsze filtrowanie)
	EMA_Filter_t dist_ema;
	EMA_Init(&dist_ema, 0.5f);

	// Start ADC nie jest potrzebny tutaj w trybie Single, będziemy startować w pętli
	// HAL_ADC_Start(&hadc1);

	// Inicjalizacja kontrolera PID
	ServoPID_Controller pid;
	ServoPID_Init(&pid);

	// Zmienne pomocnicze do walidacji odczytów
	float prev_valid_dist = 145.0f;
	int invalid_count = 0;

	// Inicjalizacja bufora błędu
	for(int i=0; i<AVG_ERR_SAMPLES; i++) err_buffer[i] = 0.0f;

	/* Infinite loop */
	for (;;) {
		loop_counter++;

		// Obsługa komend UART
		if (cmd_received) {
			if (rx_buffer[0] == 'S' && rx_buffer[1] == ':') {
				g_setpoint = atof((char*) &rx_buffer[2]); // S:Setpoint
				if (g_setpoint < 0.0f)
					g_setpoint = 0.0f;
				if (g_setpoint > 290.0f)
					g_setpoint = 290.0f;
			} else if (rx_buffer[0] == 'P' && rx_buffer[1] == ':')
				g_Kp = atof((char*) &rx_buffer[2]);
			else if (rx_buffer[0] == 'I' && rx_buffer[1] == ':')
				g_Ki = atof((char*) &rx_buffer[2]);
			else if (rx_buffer[0] == 'D' && rx_buffer[1] == ':')
				g_Kd = atof((char*) &rx_buffer[2]);
			// Komendy Kalibracji
			// Format: "CAL0:50.0,0.0" -> Punkt 0: Surowy=50.0, Rzeczywisty=0.0
			else if (rx_buffer[0] == 'C' && rx_buffer[1] == 'A' && rx_buffer[2] == 'L') {
				int cal_idx = rx_buffer[3] - '0'; // Indeks punktu
				if (cal_idx >= 0 && cal_idx < 5) {
					char *comma = strchr((char*) &rx_buffer[5], ',');
					if (comma != NULL) {
						*comma = '\0';
						float raw_val = atof((char*) &rx_buffer[5]);
						float actual_val = atof(comma + 1);

						// Aktualizacja w bibliotece kalibracyjnej
						Calibration_UpdatePoint(cal_idx, raw_val, actual_val);

						// Sprawdzenie czy mamy komplet punktów
						if (Calibration_IsReady()) {
							sprintf(msg, "[CAL] ✅ All points received! System READY!\r\n");
							HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
						} else {
							// Potwierdzenie przyjęcia punktu
							int r_i = (int) raw_val;
							int r_d = (int) ((raw_val - r_i) * 10);
							int a_i = (int) actual_val;
							int a_d = (int) ((actual_val - a_i) * 10);

							sprintf(msg, "[CAL] Point %d: RAW=%d.%d -> POS=%d.%d (%d/5)\r\n", cal_idx, r_i, abs(r_d),
									a_i, abs(a_d), __builtin_popcount(Calibration_GetReceivedPointsMask()));
							HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
						}
					}
				}
			} else if (rx_buffer[0] == 'M' && rx_buffer[1] == ':') {
				// M:0 -> GUI, M:1 -> Potencjometr
				control_mode = (uint8_t) atoi((char*) &rx_buffer[2]);
				HAL_UART_Transmit(&huart3, (uint8_t*) (control_mode ? "MODE: ANALOG\r\n" : "MODE: GUI\r\n"), 14, 100);
			}

			cmd_received = 0;
		}

		// Odczyt dystansu w trybie continuous
		distance = readRangeContinuousMillimeters(&distanceStr);

		// --- Obsługa Potencjometru ---
		// Wyzwalanie ręczne (Single Mode)
		HAL_ADC_Start(&hadc1); // Start konwersji
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			g_adc_raw = HAL_ADC_GetValue(&hadc1);
			float pot_setpoint = (float)g_adc_raw * 0.070818f; 
			g_pot_setpoint = pot_setpoint; 
			
			// Jeśli tryb Analog, nadpisz setpoint
			if (control_mode == 1) {
				// Lekki filtr na setpoint z potencjometru, żeby nie skakał
				static float pot_ema = 0.0f;
				// Inicjuj przy pierwszym użyciu
				if (pot_ema == 0.0f) pot_ema = pot_setpoint;
				
				pot_ema = 0.1f * pot_setpoint + 0.9f * pot_ema;
				g_setpoint = pot_ema;
			}
		}
		// HAL_ADC_Stop nie jest konieczne w Single Mode (sam się zatrzymuje po EOC), ale dobre dla porządku
		HAL_ADC_Stop(&hadc1);

		if (distance >= 8190) {
			// 8190/8191 = Hardware error / Out of range
			distance = (uint16_t) prev_valid_dist;
		} else {
			// Accept the measurement even if Status != 0
			prev_valid_dist = (float) distance;
		}

		// Filtracja Medianowa (usuwanie szpilek z surowego odczytu)
		float dist_median = MedianFilter_Apply(&median_filter, (float) distance);

		// Filtracja Skoków (Spike Filter)
		if (loop_counter > 10) { // Allow startup
			float jump = dist_median - prev_valid_dist;
			if (jump < 0)
				jump = -jump;

			if (jump > 25.0f) {
				if (invalid_count < 5) {
					dist_median = prev_valid_dist; // Ignore this sample, use previous
					invalid_count++;
				} else {
					// Persisted long enough, accept it
					invalid_count = 0;
					prev_valid_dist = dist_median;
				}
			} else {
				prev_valid_dist = dist_median;
				invalid_count = 0;
			}
		} else {
			prev_valid_dist = dist_median;
		}

		if (!Calibration_IsReady()) {
			// Keep sending status message every 2 seconds
			static uint32_t last_cal_msg = 0;
			if (HAL_GetTick() - last_cal_msg > 2000) {
				sprintf(msg, "[WAITING] Calibration needed (%d/5 points)\r\n",
						__builtin_popcount(Calibration_GetReceivedPointsMask()));
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
				last_cal_msg = HAL_GetTick();
			}

			static uint8_t cal_throttle = 0;
			cal_throttle++;

			if (cal_throttle >= 2) { // Send every 2nd loop (~20ms)
				cal_throttle = 0;

				char cal_buffer[64];
				int cal_len = sprintf(cal_buffer, "D:%d;A:0;F:%d;E:0;S:%d", (int) dist_median, (int) dist_median,
						distanceStr.rangeStatus);
				uint8_t cal_crc = CalculateCRC8(cal_buffer, cal_len);
				sprintf(msg, "%s;C:%02X\r\n", cal_buffer, cal_crc);
				HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 10);
			}

			HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
			osDelay(10);
			continue; // Skip PID and servo control
		}

#if USE_CALIBRATION
		float dist_calibrated = Calibration_Interpolate(dist_median);
#else
      float dist_calibrated = dist_median;
#endif

		// Lekki filtr EMA (alpha=0.85)
		float filtered_dist = EMA_Update(&dist_ema, dist_calibrated);

		distance = (uint16_t) dist_calibrated;

		float current_error = g_setpoint - filtered_dist;
		float pid_error = current_error;

		// --- Obliczanie Średniego Błędu (Rolling Average) ---
		float current_abs_err = (current_error < 0) ? -current_error : current_error;
		
		err_sum -= err_buffer[err_idx];       // Odejmij najstarszą próbkę
		err_buffer[err_idx] = current_abs_err; // Zapisz nową
		err_sum += current_abs_err;           // Dodaj nową
		
		err_idx++;
		if(err_idx >= AVG_ERR_SAMPLES) err_idx = 0;
		
		float avg_error = err_sum / AVG_ERR_SAMPLES;

		// Strefa nieczułości dla uchybu (stabilizacja w punkcie równowagi)
		if (pid_error > -2.0f && pid_error < 2.0f) {
			pid_error = 0.0f;
		}

		float pid_angle = ServoPID_Compute(&pid, pid_error, filtered_dist);
		UpdateErrorLEDs_5LED(current_error); // Wizualizacja 5-LED
		// TestLED();

		static float prev_servo_angle = SERVO_CENTER;
		float max_angle_change = 180.0f;

		float angle_diff = pid_angle - prev_servo_angle;
		if (angle_diff > max_angle_change) {
			pid_angle = prev_servo_angle + max_angle_change;
		} else if (angle_diff < -max_angle_change) {
			pid_angle = prev_servo_angle - max_angle_change;
		}

		if (pid_angle < SERVO_MIN_LIMIT)
			pid_angle = SERVO_MIN_LIMIT;
		if (pid_angle > SERVO_MAX_LIMIT)
			pid_angle = SERVO_MAX_LIMIT;

		prev_servo_angle = pid_angle;

		static float ema_servo_angle = SERVO_CENTER;
		float alpha_servo = 0.55f;
		ema_servo_angle = alpha_servo * pid_angle + (1.0f - alpha_servo) * ema_servo_angle;
		float smoothed_angle = ema_servo_angle;

		static float last_sent_angle = SERVO_CENTER;
		float angle_change =
				(smoothed_angle > last_sent_angle) ?
						(smoothed_angle - last_sent_angle) : (last_sent_angle - smoothed_angle);

		if (angle_change >= SERVO_ANGLE_DEADBAND) {
			// Significant change - update servo
			SetServoAngle(smoothed_angle);
			last_sent_angle = smoothed_angle;
		}

		char data_buffer[64];
		// D:Dist; A:Angle; F:Filtered; E:Error; V:AvgError; S:Status; Z:Setpoint
		int len = sprintf(data_buffer, "D:%d;Z:%d;A:%d;F:%d;E:%d;V:%d;S:%d", distance, (int)g_setpoint, (int) smoothed_angle, (int) filtered_dist,
				(int) current_error, (int) avg_error, distanceStr.rangeStatus);

		uint8_t out_crc = CalculateCRC8(data_buffer, len);

		sprintf(msg, "%s;C:%02X\r\n", data_buffer, out_crc);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 10); // Timeout 10ms

		osDelay(10); // Loop delay
	}
  /* USER CODE END StartControlTask */
}

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

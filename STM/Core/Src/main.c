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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "VL53L0X.h"
#include <stdio.h>
#include <stdlib.h> // for atof

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// --- Simple EMA Filter for Derivative Smoothing ---
typedef struct {
	float alpha;       // Smoothing factor (0-1, smaller = smoother)
	float filtered_value;
	uint8_t initialized;
} EMA_Filter_t;

// --- PID Controller ---
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float prevError;
	float integral;
	float prevMeasurement;  // For Derivative-on-Measurement
	EMA_Filter_t d_filter;  // Low-pass filter for derivative term
} ServoPID_Controller;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Parametry Systemu ---
#define SETPOINT_DEFAULT   100.0f
#define BALL_RADIUS        20
#define PID_DT_MS          30

// --- Limity Serwa ---
#define SERVO_CENTER       100.0f
#define SERVO_MIN_LIMIT    80.0f
#define SERVO_MAX_LIMIT    120.0f

// --- Konfiguracja Sprzętowa Serwa ---
#define SERVO_MIN_CCR      500
#define SERVO_MAX_CCR      2500
#define SERVO_MAX_ANGLE    200

// --- Tryb Testowy ---
#define SENSOR_TEST_MODE   0  // 1 aby włączyć logowanie surowych danych czujnika

// --- Filtry PID ---
#define D_DEADBAND         3.0f   // mm - ignoruj zmiany mniejsze niż ta wartość (zwiększone z 1.5)
#define D_FILTER_ALPHA     0.25f  // EMA alpha dla składowej D (zmniejszone z 0.4 dla gładszego działania)

// --- Servo Deadband (Anti-Buzzing) ---
#define SERVO_ANGLE_DEADBAND  0.2f  // stopnie - zmniejszone z 0.3 dla szybszej reakcji
#define SERVO_SMOOTHING_SIZE  5     // zmniejszone z 7 dla mniejszego lag
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
uint16_t distance;
char msg[64];

// Zmienne do odbioru UART
uint8_t rx_byte;
uint8_t rx_buffer[64];
volatile uint8_t rx_idx = 0;
volatile uint8_t cmd_received = 0;

// Zmienne Globalne Sterowania
// Ball position range: 0-290mm (0 = ball at start, 290 = ball at end)
volatile float g_setpoint = 145.0f; // Middle of beam (290/2)
volatile float g_Kp = -0.40f;  // Zwiększone z -0.29 - bardziej agresywna reakcja
volatile float g_Ki = -0.004f;
volatile float g_Kd = -1.8f;  // Zwiększone z -1.2 - silniejsza antycypacja przyspieszenia

// --- Auto-Calibration Variables ---
volatile uint8_t calibration_mode = 0;  // 0 = normal, 1 = calibrating
volatile float cal_raw_min = 9999.0f;   // Minimum raw sensor value during calibration
volatile float cal_raw_max = 0.0f;      // Maximum raw sensor value during calibration
volatile uint32_t cal_start_time = 0;   // Timestamp when calibration started

// Current calibration values (defaults, will be updated by auto-cal)
volatile float sensor_min = 50.0f;      // Raw sensor value at beam START (0mm)
volatile float sensor_max = 220.0f;     // Raw sensor value at beam END (290mm)
volatile float sensor_middle = 115.0f;  // Raw sensor value at beam MIDDLE


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

void AdaptiveEMA_Init(AdaptiveEMA_t *ema, float min_alpha, float max_alpha, float threshold);
float AdaptiveEMA_Filter(AdaptiveEMA_t *ema, float measurement);

void EMA_Init(EMA_Filter_t *ema, float alpha);
float EMA_Update(EMA_Filter_t *ema, float new_value);

// Forward declarations
void SetServoAngle(float angle);
void ServoPID_Init(ServoPID_Controller *pid);
float ServoPID_Compute(ServoPID_Controller *pid, float error, float measurement);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (rx_byte == '\n' || rx_byte == '\r') {
			rx_buffer[rx_idx] = '\0';
			if (rx_idx > 0) cmd_received = 1;
			rx_idx = 0;
		} else {
			if (rx_idx < 63) {
				rx_buffer[rx_idx++] = rx_byte;
			}
		}
		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	}
}
// --- Median Filter ---
#define MEDIAN_WINDOW_SIZE 5
typedef struct {
	float buffer[MEDIAN_WINDOW_SIZE];
	uint8_t index;
	uint8_t count;
} MedianFilter_t;

void MedianFilter_Init(MedianFilter_t *filter) {
	filter->index = 0;
	filter->count = 0;
	for (int i = 0; i < MEDIAN_WINDOW_SIZE; i++) {
		filter->buffer[i] = 0.0f;
	}
}

float MedianFilter_Apply(MedianFilter_t *filter, float v) {
	// Insert new value
	filter->buffer[filter->index] = v;
	filter->index = (filter->index + 1) % MEDIAN_WINDOW_SIZE;
	if (filter->count < MEDIAN_WINDOW_SIZE) {
		filter->count++;
	}

	// Create a temporary array for sorting
	float sorted[MEDIAN_WINDOW_SIZE];
	for (int i = 0; i < filter->count; i++) {
		sorted[i] = filter->buffer[i];
	}

	// Simple Bubble Sort (optimized for small size 9)
	for (int i = 0; i < filter->count - 1; i++) {
		for (int j = 0; j < filter->count - i - 1; j++) {
			if (sorted[j] > sorted[j + 1]) {
				float temp = sorted[j];
				sorted[j] = sorted[j + 1];
				sorted[j + 1] = temp;
			}
		}
	}

	// Return median
	if (filter->count % 2 == 0) {
		return (sorted[filter->count / 2 - 1] + sorted[filter->count / 2]) / 2.0f;
	} else {
		return sorted[filter->count / 2];
	}
}

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
	float error = (measurement > ema->value) ? (measurement - ema->value) : (ema->value - measurement);
	float factor = error / ema->threshold;
	if (factor > 1.0f)
		factor = 1.0f;
	float alpha = ema->min_alpha + (ema->max_alpha - ema->min_alpha) * factor;
	ema->value = alpha * measurement + (1.0f - alpha) * ema->value;
	return ema->value;
}

// --- Simple EMA Filter Implementation ---
void EMA_Init(EMA_Filter_t *ema, float alpha) {
	ema->alpha = alpha;
	ema->filtered_value = 0.0f;
	ema->initialized = 0;
}

float EMA_Update(EMA_Filter_t *ema, float new_value) {
	if (!ema->initialized) {
		ema->filtered_value = new_value;
		ema->initialized = 1;
		return new_value;
	}
	ema->filtered_value = ema->alpha * new_value + (1.0f - ema->alpha) * ema->filtered_value;
	return ema->filtered_value;
}


// --- 1-Euro Filter Implementation ---
// Based on: http://cristal.univ-lille.fr/~casiez/1euro/
typedef struct {
    float min_cutoff; // Minimum cutoff frequency (Hz)
    float beta;       // Speed coefficient
    float d_cutoff;   // Cutoff frequency for derivative (Hz)
    float x_prev;     // Previous value
    float dx_prev;    // Previous derivative
    uint32_t t_prev;  // Previous timestamp (ms)
    int first_run;    // Flag for first run
} OneEuroFilter_t;

void OneEuro_Init(OneEuroFilter_t *f, float min_cutoff, float beta, float d_cutoff) {
    f->min_cutoff = min_cutoff;
    f->beta = beta;
    f->d_cutoff = d_cutoff;
    f->x_prev = 0.0f;
    f->dx_prev = 0.0f;
    f->t_prev = 0;
    f->first_run = 1;
}

float OneEuro_Update(OneEuroFilter_t *f, float x, uint32_t t_now) {
    if (f->first_run) {
        f->x_prev = x;
        f->dx_prev = 0.0f;
        f->t_prev = t_now;
        f->first_run = 0;
        return x;
    }

    float dt = (t_now - f->t_prev) / 1000.0f; // Convert ms to seconds
    if (dt <= 0.0f) return f->x_prev; // Zero or negative time diff (shouldn't happen)
    
    // 1. Compute derivative (dx)
    float dx = (x - f->x_prev) / dt;

    // 2. Filter derivative (Low Pass)
    float rc_d = 1.0f / (2.0f * 3.14159f * f->d_cutoff);
    float alpha_d = 1.0f / (1.0f + rc_d / dt);
    float dx_hat = dx * alpha_d + f->dx_prev * (1.0f - alpha_d);

    // 3. Compute dynamic cutoff based on speed
    float cutoff = f->min_cutoff + f->beta * (dx_hat > 0 ? dx_hat : -dx_hat); // abs(dx)

    // 4. Filter signal (Low Pass with dynamic cutoff)
    float rc = 1.0f / (2.0f * 3.14159f * cutoff);
    float alpha = 1.0f / (1.0f + rc / dt);
    float x_hat = x * alpha + f->x_prev * (1.0f - alpha);

    // Update state
    f->x_prev = x_hat;
    f->dx_prev = dx_hat;
    f->t_prev = t_now;

    return x_hat;
}

// --- PID Implementation ---
void ServoPID_Init(ServoPID_Controller *pid) {
	pid->Kp = 0.0f;
	pid->Ki = 0.0f;
	pid->Kd = 0.0f;
	pid->prevError = 0.0f;
	pid->integral = 0.0f;
	pid->prevMeasurement = 0.0f;
	EMA_Init(&pid->d_filter, D_FILTER_ALPHA);  // Initialize derivative filter
}



float ServoPID_Compute(ServoPID_Controller *pid, float error, float measurement) {
	// Update PID terms from global variables (UART control)
	pid->Kp = g_Kp;
	pid->Ki = g_Ki;
	pid->Kd = g_Kd;

	// === P TERM ===
	float P = pid->Kp * error;

	// === I TERM (Anti-windup - EXISTING CODE) ===
	// IMPROVED ANTI-WINDUP: Only accumulate integral if output is NOT saturated
	// This prevents integral buildup when servo is at physical limits (65-135°)
	float tentative_output = SERVO_CENTER + P + (pid->Ki * pid->integral);
	
	// Track how long we've been stuck at limits
	static uint8_t stuck_at_limit_counter = 0;
	
	// Use margin: only accumulate if we're at least 2° away from limits
	// This prevents buzzing at exact limit values
	if (tentative_output >= (SERVO_MIN_LIMIT + 2.0f) && tentative_output <= (SERVO_MAX_LIMIT - 2.0f)) {
	    pid->integral += error;
	    stuck_at_limit_counter = 0; // Reset counter when not at limit
	} else {
	    // At or near limits - decay integral slowly to allow recovery
	    pid->integral *= 0.9f; // Faster decay: 10% per iteration (was 5%)
	    
	    // If stuck at limit for >10 iterations, RESET integral completely
	    stuck_at_limit_counter++;
	    if (stuck_at_limit_counter > 10) {
	        pid->integral = 0.0f; // Hard reset to stop buzzing
	        stuck_at_limit_counter = 0;
	    }
	}
	
	// Limit integral to prevent extreme values
	if (pid->integral > 5000.0f) pid->integral = 5000.0f;
	if (pid->integral < -5000.0f) pid->integral = -5000.0f;
    
	float I = pid->Ki * pid->integral;

	// === D TERM - NEW IMPLEMENTATION ===
	float D = 0.0f;
	
	// METHOD 1: Derivative-on-Measurement instead of Derivative-on-Error
	// This eliminates derivative kick when setpoint changes
	// and works on filtered signal (smoother than error)
	// Negative sign because we want to oppose change in measurement
	float raw_derivative = -(measurement - pid->prevMeasurement);
	
	// METHOD 2: Deadband - ignore tiny changes (<D_DEADBAND mm per iteration)
	// Eliminates D response to sensor noise
	if (raw_derivative > -D_DEADBAND && raw_derivative < D_DEADBAND) {
		raw_derivative = 0.0f;
	}
	
	// METHOD 3: Low-pass filter on derivative (EMA with configurable alpha)
	// Smooths out remaining noise without excessive lag
	float filtered_derivative = EMA_Update(&pid->d_filter, raw_derivative);
	
	D = pid->Kd * filtered_derivative;
	
	// Update state for next iteration
	pid->prevMeasurement = measurement;
	pid->prevError = error;

	// === FINAL OUTPUT ===
	float output = SERVO_CENTER + P + I + D;
	return output;
}





void SetServoAngle(float angle) {
	// Mapping: 0 deg -> 500us (2.5%), 180 deg -> 2500us (12.5%)
	// ARR = 19999 (20ms period)
	// Pulse = 500 + (angle / 180.0) * 2000
    // Simplified mapping for typical SG90/MG996R:
    // 0 deg = ~500 (Compare value)
    // 180 deg = ~2500 (Compare value)
    
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint32_t pulse_length = (uint32_t)(500.0f + (angle / 180.0f) * 2000.0f);
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
  /* USER CODE BEGIN 2 */

	// Start PWM for Servo (TIM3 Channel 1)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
	// --- POWER STABILIZATION DELAY ---
	// Wait 2 seconds to let power supply capaictors charge and voltage stabilize
	// This helps prevent Brownout Resets on servo startup
	HAL_UART_Transmit(&huart3, (uint8_t*) "Stabilizing Power...\r\n", 22, 100);
	HAL_Delay(2000); 

	// --- GENTLE STARTUP ---
	HAL_UART_Transmit(&huart3, (uint8_t*) "Setting Servo to CENTER\r\n", 25, 100);
	SetServoAngle(SERVO_CENTER); // Move to 100 deg once
	
	// Optional: Flash Green LED to indicate success (we survived the move)
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);

	// Inicjalizacja VL53L0X
	statInfo_t_VL53L0X distanceStr;

	// CRITICAL: Wait for I2C bus and sensor to stabilize
	// VL53L0X requires ~2ms boot time after power-on, plus I2C settling
	HAL_Delay(100); // 100ms to ensure sensor is ready

	HAL_UART_Transmit(&huart3, (uint8_t*) "Initializing VL53L0X...\r\n", 25, 100);
	if (!initVL53L0X(1, &hi2c1)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Init Failed!\r\n", 22, 100);
	} else {
		HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Init Success!\r\n", 23, 100);
	}


	// Ustaw timeout dla odczytów - musi być większy niż timing budget
	
	// ========================================================================
	// MODIFIED CONFIGURATION - Aiming for Stability & Accuracy (Fixing S:4/S:6)
	// ========================================================================
	
	// ========================================================================
	// MODIFIED CONFIGURATION - "LONG RANGE" / WEAK SIGNAL OPTIMIZATION
	// Fixing S:4 (Phase), S:6 (Sigma), S:11 (SNR) on dark/small targets
	// ========================================================================
	
	// Signal Rate: 0.01 - Allow VERY weak signals (fixes S:11 SNR noise)
	setSignalRateLimit(0.01f);
	
	// VCSEL: 18/14 - Long Range mode (Maximum sensitivity)
	setVcselPulsePeriod(VcselPeriodPreRange, 18);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14);
	
	// Phase Limits: Fully relaxed
	setPhaseCalibrationLimits(0x00, 0xFF, 0x00, 0xFF);
	
	// Timing Budget: 80ms (~12 Hz) - Long integration time for accuracy
	setMeasurementTimingBudget(80000);
	setTimeout(200); 

	// Start continuous mode
	startContinuous(0);



	HAL_UART_Transmit(&huart3, (uint8_t*) "VL53L0X Ready! Sending data...\r\n", 32, 100);

	// Start Rx Interrupt
	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t loop_counter = 0;

	// Inicjalizacja filtra 1-Euro
	OneEuroFilter_t one_euro;
	OneEuro_Init(&one_euro, 0.5f, 0.003f, 1.0f);

	// Inicjalizacja filtra medianowego
	MedianFilter_t median_filter;
	MedianFilter_Init(&median_filter);

	// Inicjalizacja PID
	ServoPID_Controller pid;
	ServoPID_Init(&pid);

    // Startup with safe value
    float prev_valid_dist = 145.0f; 
    int invalid_count = 0;

	while (1) {
		loop_counter++;

		// --- Handle Incoming Commands ---
		if (cmd_received) {
			if (rx_buffer[0] == 'S' && rx_buffer[1] == ':') {
				g_setpoint = atof((char*)&rx_buffer[2]);
				if (g_setpoint < 0.0f) g_setpoint = 0.0f;
				if (g_setpoint > 290.0f) g_setpoint = 290.0f;
			} 
			else if (rx_buffer[0] == 'P' && rx_buffer[1] == ':') g_Kp = atof((char*)&rx_buffer[2]); 
			else if (rx_buffer[0] == 'I' && rx_buffer[1] == ':') g_Ki = atof((char*)&rx_buffer[2]); 
			else if (rx_buffer[0] == 'D' && rx_buffer[1] == ':') g_Kd = atof((char*)&rx_buffer[2]);
			
			cmd_received = 0;
		}



		// Odczyt dystansu w trybie continuous (szybki)
		// Odczyt dystansu w trybie continuous
		// Removed "skip if same" logic to prevent freezing on stable values
		distance = readRangeContinuousMillimeters(&distanceStr);

        // --- FILTER 0: Sensor Status Check ---
        // RELAXED: Accept all inputs that are not implementation-defined error codes (819x)
        // This allows S:6 (Sigma Fail) to pass through if the distance looks valid.
        
        if (distance >= 8190) {
             // 8190/8191 = Hardware error / Out of range
             distance = (uint16_t)prev_valid_dist; 
        } else {
             // Accept the measurement even if Status != 0
             prev_valid_dist = (float)distance;
        }
 

		// 1. Filtracja Medianowa (usuwanie szpilek z surowego odczytu)
		// Rzutowanie na float dla zachowania precyzji w medianie
		float dist_median = MedianFilter_Apply(&median_filter, (float) distance);

        // --- FILTER 1: Jump Rejection (Spike Filter) ---
        // If jump > 100mm in one step (unlikely for a ball), ignore unless it persists.
        if (loop_counter > 10) { // Allow startup
             float jump = dist_median - prev_valid_dist;
             if (jump < 0) jump = -jump;
             
             if (jump > 50.0f && distanceStr.rangeStatus == 0) { // 5cm jump strict limit
                  if (invalid_count < 5) {
                      dist_median = prev_valid_dist; // Ignore this sample, use previous
                      invalid_count++;
                  } else {
                      // Persisted long enough, accept it (maybe moved fast)
                      invalid_count = 0;
                      prev_valid_dist = dist_median;
                  }
             } else {
                 if (distanceStr.rangeStatus == 0) {
                    prev_valid_dist = dist_median;
                    invalid_count = 0;
                 }
             }
        } else {
             prev_valid_dist = dist_median;
        }


		// 2. Kalibracja: Raw sensor → Position on beam
		// FINAL calibration based on actual measurements:
		// dist_median at start: 50mm → position 0mm
		// dist_median at end: 240mm → position 290mm
		// Sensor range: 190mm (240-50)
		// Beam length: 290mm
		// Scaling: 290/190 = 1.5263
		
// --- 5-Point Calibration Table ---
// Maps RAW sensor reading (x) to ACTUAL beam position in mm (y)
// Must be sorted by RawValue (Start -> End)
typedef struct {
	float raw_val;
	float actual_pos;
} CalPoint_t;

// DEFAULT VALUES (You must update these with real readings!)
// Example: Put ball at 0mm, read "D:50". Put at 72mm, read "D:90"...
CalPoint_t cal_table[5] = {
	{ 0.0f,   0.0f },   // Point 0: Ball at Start
	{ 115.0f,   72.5f },  // Point 1: Ball at 25%
	{ 215.0f,  145.0f }, // Point 2: Ball at 50% (Middle)
	{ 265.0f,  217.5f }, // Point 3: Ball at 75%
	{ 280.0f,  290.0f }  // Point 4: Ball at End
};

float InterpolateDistance(float raw_input) {
	// Clamp to range
	if (raw_input <= cal_table[0].raw_val) return cal_table[0].actual_pos;
	if (raw_input >= cal_table[4].raw_val) return cal_table[4].actual_pos;

	// Find segment
	for (int i = 0; i < 4; i++) {
		if (raw_input >= cal_table[i].raw_val && raw_input <= cal_table[i+1].raw_val) {
			
			// Linear Interpolation: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
			float range_x = cal_table[i+1].raw_val - cal_table[i].raw_val;
			float range_y = cal_table[i+1].actual_pos - cal_table[i].actual_pos;
			
			if (range_x < 0.1f) return cal_table[i].actual_pos; // Avoid div/0
			
			float ratio = (raw_input - cal_table[i].raw_val) / range_x;
			return cal_table[i].actual_pos + (ratio * range_y);
		}
	}
	return raw_input; // Should not reach here
}



		// 2. Kalibracja: Multi-Point Interpolation
		// Use the 5-point table to fix non-linearity
		float dist_calibrated = InterpolateDistance(dist_median);









		// 3. Filtracja 1-Euro (wygładzanie skalibrowanego sygnału)
		// 3. Filtracja: DISABLED (User request: delete all lag)
		// float filtered_dist = OneEuro_Update(&one_euro, dist_calibrated, HAL_GetTick());
		float filtered_dist = dist_calibrated;

		// Update zmiennej display (chcemy widzieć skalibrowany dystans jako "Raw" w GUI, czy surowy?)
		// GUI wyświetla "D:xxx" jako Raw. Żeby widzieć poprawność kalibracji, wyślijmy skalibrowany.
		distance = (uint16_t) dist_calibrated;



		// Wysyłanie w formacie z CRC
		float current_error = g_setpoint - filtered_dist;

		// DEAD ZONE: DISABLED (User request for less lag)

		// if (current_error > -dead_zone && current_error < dead_zone) {
		//    current_error = 0.0f; 
		// }

		// --- PID Control Loop ---

		// Compute PID Output (with Derivative-on-Measurement)
		float pid_angle = ServoPID_Compute(&pid, current_error, filtered_dist);
		
		// RATE LIMITER: Servo can't change angle instantly
		// Limit max change per iteration to prevent servo lag
		// INCREASED from 3 to 5 degrees for faster response to ball movement
		static float prev_servo_angle = SERVO_CENTER;
		float max_angle_change = 20.0f; // INCREASED: No rate limiting (was 5.0f)
		
		float angle_diff = pid_angle - prev_servo_angle;
		if (angle_diff > max_angle_change) {
		    pid_angle = prev_servo_angle + max_angle_change;
		} else if (angle_diff < -max_angle_change) {
		    pid_angle = prev_servo_angle - max_angle_change;
		}
		
		// Clamp Servo Angle (Safety Limits 65 - 135)
		if (pid_angle < SERVO_MIN_LIMIT) pid_angle = SERVO_MIN_LIMIT;
		if (pid_angle > SERVO_MAX_LIMIT) pid_angle = SERVO_MAX_LIMIT;
		
		// CRITICAL: Update prev_servo_angle AFTER clamping
		// This ensures rate limiter knows actual servo position
		prev_servo_angle = pid_angle;
		
		// SMOOTHING: DISABLED (User request: delete all lag)
		static float ema_servo_angle = SERVO_CENTER;
		// float alpha_servo = 0.50f; 
		// ema_servo_angle = alpha_servo * pid_angle + (1.0f - alpha_servo) * ema_servo_angle;
		float smoothed_angle = pid_angle; // Pass-through
		
		// SERVO DEADBAND: DISABLED (Always update)
		// This might cause buzzing but removes lag
		SetServoAngle(smoothed_angle);

		
		/*
		if (angle_change >= SERVO_ANGLE_DEADBAND) {
			// Significant change - update servo
			SetServoAngle(smoothed_angle);
			last_sent_angle = smoothed_angle;
		}
		*/
		// Else: change too small, don't update (prevents buzzing) 
 

		char data_buffer[64];
		// Standard protocol: D:Dist; A:Angle; F:Filtered; E:Error; S:Status
		int len = sprintf(data_buffer, "D:%d;A:%d;F:%d;E:%d;S:%d", 
				distance, (int)smoothed_angle, (int)filtered_dist,
				(int)current_error, distanceStr.rangeStatus);
				
		uint8_t out_crc = CalculateCRC8(data_buffer, len);

		sprintf(msg, "%s;C:%02X\r\n", data_buffer, out_crc);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 10); // Timeout 10ms
		
		// RATE CONTROL: Limited by sensor TIMING BUDGET (80ms)
		// Small delay to allow UART/Interupts to process
		HAL_Delay(5);
	}



	loop_counter++;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */

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

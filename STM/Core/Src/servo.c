/**
 ******************************************************************************
 * @file    servo.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Sterownik serwomechanizmu PWM.
 *
 * Plik zawiera funkcje sterujące serwomechanizmem poprzez generowanie sygnału PWM
 * przy użyciu sprzętowego Timera STM32.
 ******************************************************************************
 */

#include "servo.h"

/**
 * @brief Inicjalizuje strukturę serwa.
 *        Konfiguruje parametry pracy serwomechanizmu, takie jak minimalna i maksymalna
 *        szerokość impulsu oraz zakres kątowy.
 * @param hservo Wskaźnik do struktury Servo_Handle_t.
 * @param htim Wskaźnik do uchwytu timera (TIM_HandleTypeDef).
 * @param channel Kanał timera (np. TIM_CHANNEL_1).
 * @param min_pulse Wartość CCR dla minimalnego kąta (0 stopni).
 * @param max_pulse Wartość CCR dla maksymalnego kąta.
 * @param max_angle Maksymalny kąt obrotu serwa.
 */
void Servo_Init(Servo_Handle_t *hservo, TIM_HandleTypeDef *htim, uint32_t channel, uint32_t min_pulse, uint32_t max_pulse, uint16_t max_angle) {
    hservo->htim = htim;
    hservo->channel = channel;
    hservo->min_pulse = min_pulse;
    hservo->max_pulse = max_pulse;
    hservo->max_angle = max_angle;
}

/**
 * @brief Ustawia kąt serwa.
 *        Przelicza podany kąt na odpowiednią szerokość impulsu PWM i aktualizuje rejestr CCR timera.
 * @param hservo Wskaźnik do struktury Servo_Handle_t.
 * @param angle Żądany kąt w stopniach (0 - max_angle).
 */
void Servo_SetAngle(Servo_Handle_t *hservo, uint16_t angle) {
    if (angle > hservo->max_angle) {
        angle = hservo->max_angle;
    }

    // Mapowanie kąta na szerokość impulsu (wartość CCR)
    // Wzór: pulse = min + (angle * (max - min) / max_angle)
    uint32_t pulse = hservo->min_pulse + ((uint32_t)angle * (hservo->max_pulse - hservo->min_pulse) / hservo->max_angle);

    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, pulse);
}

/**
 * @brief Uruchamia sygnał PWM dla serwa.
 *        Włącza wyjście PWM na skonfigurowanym kanale timera.
 * @param hservo Wskaźnik do struktury Servo_Handle_t.
 */
void Servo_Start(Servo_Handle_t *hservo) {
    HAL_TIM_PWM_Start(hservo->htim, hservo->channel);
}

/**
 * @brief Zatrzymuje sygnał PWM dla serwa.
 *        Wyłącza wyjście PWM na skonfigurowanym kanale timera.
 * @param hservo Wskaźnik do struktury Servo_Handle_t.
 */
void Servo_Stop(Servo_Handle_t *hservo) {
    HAL_TIM_PWM_Stop(hservo->htim, hservo->channel);
}

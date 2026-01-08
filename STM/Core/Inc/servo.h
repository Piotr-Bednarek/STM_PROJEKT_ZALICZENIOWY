/**
 ******************************************************************************
 * @file    servo.h
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Nagłówek biblioteki sterownika serwomechanizmu PWM.
 ******************************************************************************
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

/**
 * @brief Struktura konfiguracyjna serwomechanizmu
 */
typedef struct {
    TIM_HandleTypeDef *htim;  ///< Uchwyt do timera (np. &htim2)
    uint32_t channel;         ///< Kanał timera (np. TIM_CHANNEL_1)
    uint32_t min_pulse;       ///< Wartość CCR dla minimalnego kąta (0 stopni)
    uint32_t max_pulse;       ///< Wartość CCR dla maksymalnego kąta
    uint16_t max_angle;       ///< Maksymalny kąt serwa
} Servo_Handle_t;

/**
 * @brief Inicjalizacja struktury serwa
 * @param hservo Wskaźnik do struktury Servo_Handle_t
 * @param htim Wskaźnik do uchwytu timera
 * @param channel Kanał timera
 * @param min_pulse Wartość rejestru CCR dla 0 stopni
 * @param max_pulse Wartość rejestru CCR dla max_angle
 * @param max_angle Maksymalny kąt obrotu
 */
void Servo_Init(Servo_Handle_t *hservo, TIM_HandleTypeDef *htim, uint32_t channel, uint32_t min_pulse, uint32_t max_pulse, uint16_t max_angle);

/**
 * @brief Ustawia kąt serwa
 * @param hservo Wskaźnik do struktury Servo_Handle_t
 * @param angle Żądany kąt (0 do max_angle)
 */
void Servo_SetAngle(Servo_Handle_t *hservo, uint16_t angle);

/**
 * @brief Rozpoczyna generowanie PWM dla serwa
 * @param hservo Wskaźnik do struktury Servo_Handle_t
 */
void Servo_Start(Servo_Handle_t *hservo);

/**
 * @brief Zatrzymuje generowanie PWM dla serwa
 * @param hservo Wskaźnik do struktury Servo_Handle_t
 */
void Servo_Stop(Servo_Handle_t *hservo);

#endif /* INC_SERVO_H_ */

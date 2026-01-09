/**
 ******************************************************************************
 * @file    pid.c
 * @author  Piotr Bednarek Jan Andrzejewski Mateusz Banaszak
 * @date    Jan 8, 2026
 * @brief   Wrapper na bibliotekę CMSIS DSP PID.
 *
 * Plik zawiera funkcje inicjalizujące i obsługujące regulator PID
 * z wykorzystaniem zoptymalizowanej biblioteki matematycznej ARM (CMSIS DSP).
 ******************************************************************************
 */

#include "pid.h"

/**
 * @brief Inicjalizuje regulator PID.
 *        Ustawia współczynniki wzmocnienia oraz limity wyjścia (nasycenie).
 *        Wywołuje arm_pid_init_f32 aby zresetować stan wewnętrzny algorytmu.
 * @param pid Wskaźnik do struktury PID_Controller_t.
 * @param Kp Wzmocnienie proporcjonalne.
 * @param Ki Wzmocnienie całkujące.
 * @param Kd Wzmocnienie różniczkujące.
 * @param min_out Dolny limit wyjścia.
 * @param max_out Górny limit wyjścia.
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
    pid->instance.Kp = Kp;
    pid->instance.Ki = Ki;
    pid->instance.Kd = Kd;
    
    arm_pid_init_f32(&pid->instance, 1);
    
    pid->output_min = min_out;
    pid->output_max = max_out;
}

/**
 * @brief Oblicza wyjście regulatora PID dla zadanego uchybu.
 *        Funkcja oblicza błąd (setpoint - measured), a następnie wywołuje funkcję arm_pid_f32.
 *        Wynik jest ograniczany (nasycany) do zakresu [output_min, output_max].
 * @param pid Wskaźnik do struktury PID_Controller_t.
 * @param setpoint Wartość zadana.
 * @param measured Wartość mierzona.
 * @return Wartość sterująca (wyjście regulatora) po saturacji.
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    
    float32_t out = arm_pid_f32(&pid->instance, error);
    
    if (out > pid->output_max) {
        out = pid->output_max;
    } else if (out < pid->output_min) {
        out = pid->output_min;
    }
    
    return out;
}

/**
 * @brief Resetuje stan regulatora PID.
 *        Zeruje całkę i historię błędów.
 * @param pid Wskaźnik do struktury PID_Controller_t.
 */
void PID_Reset(PID_Controller_t *pid) {
    arm_pid_init_f32(&pid->instance, 1);
}

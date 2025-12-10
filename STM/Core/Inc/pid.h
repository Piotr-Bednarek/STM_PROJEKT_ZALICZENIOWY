#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#define ARM_MATH_CM7
#include "arm_math.h"

typedef struct {
    arm_pid_instance_f32 instance;
    float output_min;
    float output_max;
} PID_Controller_t;

/**
 * @brief Inicjalizacja regulatora PID z wykorzystaniem CMSIS DSP
 * @param pid Wskaźnik na strukturę kontrolera
 * @param Kp Wzmocnienie proporcjonalne
 * @param Ki Wzmocnienie całkujące
 * @param Kd Wzmocnienie różniczkujące
 * @param min_out Minimalna wartość wyjścia (nasycenie)
 * @param max_out Maksymalna wartość wyjścia (nasycenie)
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float min_out, float max_out);

/**
 * @brief Oblicza wyjście regulatora PID
 * @param pid Wskaźnik na strukturę kontrolera
 * @param setpoint Wartość zadana (cel)
 * @param measured Wartość mierzona (aktualna)
 * @return Wartość sterująca (wyjście regulatora)
 */
float PID_Compute(PID_Controller_t *pid, float setpoint, float measured);

/**
 * @brief Resetuje stan regulatora (całkę i poprzednie błędy)
 * @param pid Wskaźnik na strukturę kontrolera
 */
void PID_Reset(PID_Controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */

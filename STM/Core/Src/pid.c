#include "pid.h"

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
    // Ustawienie współczynników
    pid->instance.Kp = Kp;
    pid->instance.Ki = Ki;
    pid->instance.Kd = Kd;
    
    // Inicjalizacja biblioteczna (obliczenie A0, A1, A2) - resetState = 1
    arm_pid_init_f32(&pid->instance, 1);
    
    pid->output_min = min_out;
    pid->output_max = max_out;
}

float PID_Compute(PID_Controller_t *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    
    // Obliczenie PID przy użyciu funkcji CMSIS DSP
    float32_t out = arm_pid_f32(&pid->instance, error);
    
    // Nasycenie wyjścia (Saturation)
    if (out > pid->output_max) {
        out = pid->output_max;
    } else if (out < pid->output_min) {
        out = pid->output_min;
    }
    
    return out;
}

void PID_Reset(PID_Controller_t *pid) {
    arm_pid_init_f32(&pid->instance, 1);
}

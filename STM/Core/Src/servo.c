#include "servo.h"

void Servo_Init(Servo_Handle_t *hservo, TIM_HandleTypeDef *htim, uint32_t channel, uint32_t min_pulse, uint32_t max_pulse, uint16_t max_angle) {
    hservo->htim = htim;
    hservo->channel = channel;
    hservo->min_pulse = min_pulse;
    hservo->max_pulse = max_pulse;
    hservo->max_angle = max_angle;
}

void Servo_SetAngle(Servo_Handle_t *hservo, uint16_t angle) {
    if (angle > hservo->max_angle) {
        angle = hservo->max_angle;
    }

    // Mapowanie kąta na szerokość impulsu (wartość CCR)
    // Wzór: pulse = min + (angle * (max - min) / max_angle)
    uint32_t pulse = hservo->min_pulse + ((uint32_t)angle * (hservo->max_pulse - hservo->min_pulse) / hservo->max_angle);

    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, pulse);
}

void Servo_Start(Servo_Handle_t *hservo) {
    HAL_TIM_PWM_Start(hservo->htim, hservo->channel);
}

void Servo_Stop(Servo_Handle_t *hservo) {
    HAL_TIM_PWM_Stop(hservo->htim, hservo->channel);
}

#pragma once
#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"

#define DOMAIN 3
#define E_MAX 1000
#define DE_MAX 1000
#define KP_CHANGE 0
#define KI_CHANGE 0
#define KD_CHANGE 0
#define ENCODER 242.5218
#define SAMPLE_PERIOD 0.1

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double error_last;
    double error_last_last;
    int32_t output;
    int32_t output_max;
    int32_t output_min;
}pid_obj;

typedef struct{
    pid_obj pid;
    TIM_HandleTypeDef *pwm_htim;
    uint32_t pwm_channel;
    uint16_t rotate_pin_1;
    uint16_t rotate_pin_2;
    GPIO_TypeDef * rotate_port;
    double setVel;
    TIM_HandleTypeDef *encoder_htim;
}wheel_obj;

void pid_output(pid_obj* pid, double error);
void fuzzy_pid(pid_obj* pid, double error);
void wheel_init(wheel_obj* wheel, TIM_HandleTypeDef *p_htim, uint32_t channel, uint16_t pin1, uint16_t pin2, GPIO_TypeDef *port, TIM_HandleTypeDef *e_htim);
void wheel_reverse(wheel_obj *wheel);
void wheel_pwm(wheel_obj *wheel, double error);
double wheel_getspeed(wheel_obj *wheel);

#endif // PID_H

#include "pid.h"
#include <math.h>

const short int p_rule[7][7] = {
    3, 3, 2, 2, 1, 0, 0,
    3, 3, 2, 2, 1, 0, -1,
    2, 2, 2, 1, 0, -1, -1,
    2, 2, 1, 0, -1, -2, -2,
    1, 1, 0, -1, -1, -2, -2,
    1, 0, -1, -2, -2, -2, -3,
    0, 0, -2, -2, -2, -3, -3
};

const short int i_rule[7][7] = {
    -3, -3, -2, -2, -1, 0, 0,
    -3, -3, -2, -1, -1, 0, 0,
    -3, -2, -1, -1, 0, 1, 1,
    -2, -2, -1, 0, 1, 2, 2,
    -2, -1, 0, 1, 1, 2, 3,
    0, 0, 1, 1, 2, 3, 3,
    0, 0, 1, 2, 2, 3, 3
};

const short int d_rule[7][7] = {
    1, -1, -3, -3, -3, -2, 1,
    1, -1, -3, -2, -2, -1, 0,
    0, -1, -2, -2, -1, -1, 0,
    0, -1, -1, -1, -1, -1, 0,
    0, 0, 0, 0, 0, 0, 0,
    3, -1, 1, 1, 1, 1, 3,
    3, 2, 2, 2, 1, 1, 3
};

void pid_output(pid_obj* pid, double error)
{
    double change;
    change = pid->Kp * (error - pid->error_last) + pid->Ki * error + pid->Kd * (error - 2 * pid->error_last + pid->error_last_last);
    pid->error_last_last = pid->error_last;
    pid->error_last = error;
    pid->output += change;
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }
    
}

void wheel_init(wheel_obj* wheel, TIM_HandleTypeDef *p_htim, uint32_t channel, uint16_t pin1, uint16_t pin2, GPIO_TypeDef *port, TIM_HandleTypeDef *e_htim)
{
    wheel->pid.Kp = 1;
    wheel->pid.Ki = 3;
    wheel->pid.Kd = 1;
    wheel->pid.error_last = 0;
    wheel->pid.error_last_last = 0;
    wheel->pid.output = 0;
    wheel->pid.output_max = 1000;
    wheel->pid.output_min = -1000;
    wheel->pwm_channel = channel;
    wheel->pwm_htim = p_htim;
    wheel->rotate_pin_1 = pin1;
    wheel->rotate_pin_2 = pin2;
    wheel->rotate_port = port;
    wheel->setVel = 0;
    wheel->encoder_htim = e_htim;
    HAL_TIM_PWM_Start(wheel->pwm_htim, wheel->pwm_channel);
    HAL_GPIO_WritePin(port, pin1, SET);
    HAL_GPIO_WritePin(port, pin2, RESET);
    __HAL_TIM_SetCompare(p_htim, channel, 300);
    HAL_TIM_Encoder_Start(e_htim, TIM_CHANNEL_ALL);
}

void wheel_reverse(wheel_obj *wheel)
{
    HAL_GPIO_TogglePin(wheel->rotate_port, wheel->rotate_pin_1);
    HAL_GPIO_TogglePin(wheel->rotate_port, wheel->rotate_pin_2);
}

void wheel_pwm(wheel_obj *wheel, double error)
{
    pid_output(&wheel->pid, error);
    int32_t output = wheel->pid.output;
    if(output < 0)
    {
        wheel_reverse(wheel);
        output = -output;
    }
    __HAL_TIM_SetCompare(wheel->pwm_htim, wheel->pwm_channel, (uint32_t)output);
}

double wheel_getVel(wheel_obj *wheel)
{
    double vel;
    vel = __HAL_TIM_GetCounter(wheel->encoder_htim) / ENCODER / SAMPLE_PERIOD;
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(wheel->encoder_htim))
    {
        vel = -vel;
    }
    __HAL_TIM_SetCounter(wheel->encoder_htim, 0);
    return vel;
}

void membership(double error, double derror, double* Me, double* Mde, short int* Fe, short int* Fde)
{
    double mapping;
    if (error >= E_MAX)
    {
        Me[0] = 0;
        Me[1] = 1;
        Fe[0] = DOMAIN - 1;
        Fe[1] = DOMAIN;
    }
    else if (error <= E_MAX * (-1))
    {
        Me[0] = 1;
        Me[1] = 0;
        Fe[0] = DOMAIN * (-1) + 1;
        Fe[1] = DOMAIN * (-1);
    }
    else
    {
        mapping = error * DOMAIN / E_MAX;
        Fe[0] = floor(mapping);
        Fe[1] = ceil(mapping);
        Me[0] = mapping - Fe[0];
        Me[1] = Fe[1] - mapping;
    }

    if (derror >= DE_MAX)
    {
        Mde[0] = 0;
        Mde[1] = 1;
        Fde[0] = DOMAIN - 1;
        Fde[1] = DOMAIN;
    }
    else if (derror <= DE_MAX * (-1))
    {
        Mde[0] = 1;
        Mde[1] = 0;
        Fde[0] = DOMAIN * (-1) + 1;
        Fde[1] = DOMAIN * (-1);
    }
    else
    {
        mapping = derror * DOMAIN / DE_MAX;
        Fde[0] = floor(mapping);
        Fde[1] = ceil(mapping);
        Mde[0] = mapping - Fde[0];
        Mde[1] = Fde[1] - mapping;
    }
}

void fuzzy_rule(short int* Fe, short int* Fde, short int F[][2][2])
{
    F[0][0][0] = p_rule[Fe[0] + 3][Fde[0] + 3];
    F[0][0][1] = p_rule[Fe[0] + 3][Fde[1] + 3];
    F[0][1][0] = p_rule[Fe[1] + 3][Fde[0] + 3];
    F[0][1][1] = p_rule[Fe[1] + 3][Fde[1] + 3];

    F[1][0][0] = i_rule[Fe[0] + 3][Fde[0] + 3];
    F[1][0][1] = i_rule[Fe[0] + 3][Fde[1] + 3];
    F[1][1][0] = i_rule[Fe[1] + 3][Fde[0] + 3];
    F[1][1][1] = i_rule[Fe[1] + 3][Fde[1] + 3];

    F[2][0][0] = d_rule[Fe[0] + 3][Fde[0] + 3];
    F[2][0][1] = d_rule[Fe[0] + 3][Fde[1] + 3];
    F[2][1][0] = d_rule[Fe[1] + 3][Fde[0] + 3];
    F[2][1][1] = d_rule[Fe[1] + 3][Fde[1] + 3];
}

void fuzzy_pid(pid_obj* pid, double error)
{
    double derror = error - pid->error_last;
    double Me[2], Mde[2];
    short int Fe[2], Fde[2], F[3][2][2];
    membership(error, derror, Me, Mde, Fe, Fde);
    fuzzy_rule(Fe, Fde, F);
    pid->Kp += KP_CHANGE * (Me[0] * Mde[0] * F[0][0][0] + Me[1] * Mde[0] * F[0][0][1] + Me[1] * Mde[0] * F[0][1][0] + Me[1] * Mde[1] * F[0][1][1]);
    pid->Ki += KI_CHANGE * (Me[0] * Mde[0] * F[1][0][0] + Me[1] * Mde[0] * F[1][0][1] + Me[1] * Mde[0] * F[1][1][0] + Me[1] * Mde[1] * F[1][1][1]);
    pid->Kd += KD_CHANGE * (Me[0] * Mde[0] * F[2][0][0] + Me[1] * Mde[0] * F[2][0][1] + Me[1] * Mde[0] * F[2][1][0] + Me[1] * Mde[1] * F[2][1][1]);
    if (pid->Kp < 0)
    {
        pid->Kp = 0;
    }
    if (pid->Ki < 0)
    {
        pid->Ki = 0;
    }
    if (pid->Kd < 0)
    {
        pid->Kd = 0;
    }
    pid_output(pid, error);
}
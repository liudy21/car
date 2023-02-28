#ifndef JY62_H
#define JY62_H

#include "main.h"

struct accelerate
{
    float x;
    float y;
    float z;
};

struct angular_velocity
{
    float x;
    float y;
    float z;
};

struct angle
{
    float roll;
    float pitch;
    float yaw;
};

#define JY62_BUFFER_LEN 33
#define g 9.8
#define SAMPING_INTERVAL 10
#define FILTING_INTERVAL 20

void jy62_receive(UART_HandleTypeDef* huart);

uint8_t jy62_messageRecord();

float jy62_getAccX();
float jy62_getAccY();
float jy62_getAccZ();
float jy62_getVeloX();
float jy62_getVeloY();
float jy62_getVeloZ();
float jy62_getAngRoll();
float jy62_getAngPitch();
float jy62_getAngYaw();

void jy62_init();
void jy62_acc_calibrate();
void jy62_sleep();
void jy62_horizontal();
void jy62_vertical();
uint8_t jy62_setBaud(uint16_t baudRate);


#endif
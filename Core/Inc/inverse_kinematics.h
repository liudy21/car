#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include "zigbee_edc24.h"
//#include "millis.h"
#include "math.h"

typedef struct{
    double vel_x;
    double vel_y;
    double w_z;
}vel_s;

typedef struct{
    double w_lf;
    double w_lb;
    double w_rf;
    double w_rb;
}angVel_s;

typedef struct{
    int16_t x;
    int16_t y;
    double ang;
    uint32_t time;
}pos_time_s;

#define MAX_SELF_POS_TIME 10
#define WHEEL_RATIO 0.036
#define CAR_LENTH 0.125
#define CAR_WIDTH 0.1
#define WHEEL_ANG 0.7853

vel_s car_kinematics(angVel_s* angVel);//正运动学模型
angVel_s car_inv_kinematics(vel_s* vel);//逆运动学模型
void self_position_init();
void self_position_update();
vel_s calculate_vel();

#endif
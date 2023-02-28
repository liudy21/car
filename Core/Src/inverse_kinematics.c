#include "inverse_kinematics.h"

pos_time_s self_pos_time[MAX_SELF_POS_TIME];
static uint8_t pos_idx=0;
static uint8_t data_flag;

static uint8_t pos_idx_last(uint8_t input)
{
    if(input==0)
    {
        return MAX_SELF_POS_TIME-1;
    }
    else
    {
        return input-1;
    }
}

void self_position_init()
{
    pos_idx=0;
    data_flag=0;
}

void self_position_update()
{
    //self_pos_time[pos_idx].time=millis();
    self_pos_time[pos_idx].x=getVehiclePos().x;
    self_pos_time[pos_idx].y=getVehiclePos().y;
    if(pos_idx==MAX_SELF_POS_TIME-1)
    {
        pos_idx=0;
    }
    else
    {
        pos_idx++;
        if(data_flag==0&&pos_idx>=2)
        {
            data_flag=1;
        }
    }
}

vel_s calculate_vel()
{
    vel_s tmp_vel;
    if(data_flag==0)
    {
        tmp_vel.vel_x=0;
        tmp_vel.vel_y=0;
        tmp_vel.w_z=0;
    }
    else
    {
        tmp_vel.vel_x=(double)((self_pos_time[pos_idx_last(pos_idx)].x-self_pos_time[pos_idx_last(pos_idx_last(pos_idx))].x)/(self_pos_time[pos_idx_last(pos_idx)].time)-(self_pos_time[pos_idx_last(pos_idx_last(pos_idx))].time));
        tmp_vel.vel_y=(double)((self_pos_time[pos_idx_last(pos_idx)].y-self_pos_time[pos_idx_last(pos_idx_last(pos_idx))].y)/(self_pos_time[pos_idx_last(pos_idx)].time)-(self_pos_time[pos_idx_last(pos_idx_last(pos_idx))].time));
        tmp_vel.w_z=(double)((self_pos_time[pos_idx_last(pos_idx)].ang-self_pos_time[pos_idx_last(pos_idx_last(pos_idx))].ang)/(self_pos_time[pos_idx_last(pos_idx)].time)-(self_pos_time[pos_idx_last(pos_idx_last(pos_idx))].time));
    }
    return tmp_vel;
}

vel_s car_kinematics(angVel_s* angVel)
{
    vel_s tmp_vel;
    tmp_vel.vel_x=WHEEL_RATIO*(angVel->w_lf+angVel->w_lb+angVel->w_rf+angVel->w_rb)/4;
    tmp_vel.vel_y=WHEEL_RATIO*(-angVel->w_lf+angVel->w_lb+angVel->w_rf-angVel->w_rb)/4;
    tmp_vel.w_z=WHEEL_RATIO*((-angVel->w_lf-angVel->w_lb+angVel->w_rf+angVel->w_rb)/(4*CAR_LENTH*tan(WHEEL_ANG)+CAR_WIDTH));
    return tmp_vel;
}

angVel_s car_inv_kinematics(vel_s* vel)
{
    angVel_s tmp_angvel;
    tmp_angvel.w_lf=(vel->vel_x-tan(WHEEL_ANG)*vel->vel_y-(CAR_LENTH*tan(WHEEL_ANG)+CAR_WIDTH)*vel->w_z)/WHEEL_RATIO;
    tmp_angvel.w_lb=(vel->vel_x+tan(WHEEL_ANG)*vel->vel_y-(CAR_LENTH*tan(WHEEL_ANG)+CAR_WIDTH)*vel->w_z)/WHEEL_RATIO;
    tmp_angvel.w_rf=(vel->vel_x+tan(WHEEL_ANG)*vel->vel_y+(CAR_LENTH*tan(WHEEL_ANG)+CAR_WIDTH)*vel->w_z)/WHEEL_RATIO;
    tmp_angvel.w_rb=(vel->vel_x-tan(WHEEL_ANG)*vel->vel_y+(CAR_LENTH*tan(WHEEL_ANG)+CAR_WIDTH)*vel->w_z)/WHEEL_RATIO;
    return tmp_angvel;
}
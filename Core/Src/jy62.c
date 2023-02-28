#include "jy62.h"
#include "main.h"
#include "math.h"

uint8_t jy62_flag;

uint8_t receive[JY62_BUFFER_LEN] = {0};

struct accelerate acc;
struct angular_velocity velo;
struct angle ang;

UART_HandleTypeDef* jy62_huart;

uint8_t init_command[3] = {0xFF, 0xAA, 0x52};
uint8_t calibrate_command[3] = {0xFF, 0xAA, 0x67};
uint8_t sleep_command[3] = {0xFF, 0xAA, 0x60};
uint8_t hori_command[3] = {0xFF, 0xAA, 0x65};
uint8_t verti_command[3] = {0xFF, 0xAA, 0x66};
uint8_t baud_115200_command[3] = {0xFF, 0xAA, 0x63};
uint8_t baud_9600_command[3] = {0xFF, 0xAA, 0x64};

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if(huart == jy62_huart)
    {
        u1_printf("uart receive\n");
        jy62_flag = 1;
    }
}
*/

//开始接收数据
//
//参数为USART句柄
void jy62_receive(UART_HandleTypeDef* huart)
{
    jy62_huart = huart;
    jy62_flag = 0;
    HAL_UART_Receive_DMA(jy62_huart, receive, JY62_BUFFER_LEN);
}

//处理接收的数据，存到结构体中
//返回flag表示读到了哪些包，每一位从右向左依次对应加速度、角速度和姿态角
//
uint8_t jy62_messageRecord()
{
    uint8_t pos;
    volatile uint8_t flag = 0x00;
    HAL_StatusTypeDef result;
    //u1_printf("jy message begin\n");
    for(pos=0; pos<JY62_BUFFER_LEN - 10; pos++)
    {
        if(receive[pos] == 0x55)
            break;
    }
    while(receive[pos] == 0x55)
    {
        switch(receive[pos+1])
        {
            case 0x51:
            acc.x = (short)(((short)receive[pos+3]<<8) | receive[pos+2])/32168. * 16 * g;
            acc.y = (short)(((short)receive[pos+5]<<8) | receive[pos+4])/32168. * 16 * g;
            acc.z = (short)(((short)receive[pos+7]<<8) | receive[pos+6])/32168. * 16 * g;
            flag |= 0x01;
            //u1_printf("1, %d\n", flag);
            pos += 11;
            break;

            case 0x52:
            velo.x = (short)(((short)receive[pos+3]<<8) | receive[pos+2])/32168. * 2000;
            velo.y = (short)(((short)receive[pos+5]<<8) | receive[pos+4])/32168. * 2000;
            velo.z = (short)(((short)receive[pos+7]<<8) | receive[pos+6])/32168. * 2000;
            flag |= 0x02;
            //u1_printf("2, %d\n", flag);
            pos += 11;
            break;

            case 0x53:
            ang.roll = (float)((receive[pos+3]<<8) | receive[pos+2])/32768.*180;
            ang.pitch = (float)((receive[pos+5]<<8) | receive[pos+4])/32768.*180;
            ang.yaw = (float)((receive[pos+7]<<8) | receive[pos+6])/32768.*180;
            flag |= 0x04;
            //u1_printf("3, %d\n", flag);
            pos += 11;
            break;

            default:
            for(pos += 1; pos < JY62_BUFFER_LEN;  pos++)
            {
                if(receive[pos] == 0x55)
                    break;
            }
            //u1_printf("default\n");
            break;

        }
        if(pos >= JY62_BUFFER_LEN - 10)
            break;
    }
    //for(int i=0; i<JY62_BUFFER_LEN; i++)
        //u1_printf("%d %x\n", i, receive[i]);
    //u1_printf("end, %d\n", flag);
    result = HAL_UART_Receive_DMA(jy62_huart, receive, JY62_BUFFER_LEN);
    //u1_printf("uart: %d\n", result);
    //jy62_flag = 0;
    return flag;
}

//初始化、校准

//向外界提供数据
//返回需要的值
//
float jy62_getAccX()
{
    return acc.x;
}

float jy62_getAccY()
{
    return acc.y;
}

float jy62_getAccZ()
{
    return acc.z;
}

float jy62_getVeloX()
{
    return velo.x;
}

float jy62_getVeloY()
{
    return velo.y;
}

float jy62_getVeloZ()
{
    return velo.z;
}

float jy62_getAngRoll()
{
    return ang.roll;
}

float jy62_getAngPitch()
{
    return ang.pitch;
}

float jy62_getAngYaw()
{
    return ang.yaw;
}

//各种配置函数

//角度初始化，使z轴角度归零
//
//
void jy62_init()
{
    HAL_UART_Transmit_DMA(jy62_huart, init_command, 3);
}

//校准加速度计零偏
//
//
void jy62_acc_calibrate()
{
    HAL_UART_Receive_DMA(jy62_huart, calibrate_command, 3);
}

//休眠或者解休眠
//
//
void jy62_sleep()
{
    HAL_UART_Transmit_DMA(jy62_huart, sleep_command, 3);
}

//设置为水平安装
//
//
void jy62_horizontal()
{
    HAL_UART_Transmit_DMA(jy62_huart, hori_command, 3);
}

//设置为竖直安装
//
//
void jy62_vertical()
{
    HAL_UART_Transmit_DMA(jy62_huart, verti_command, 3);
}

//设置波特率
//传入需要设定的波特率
//返回1，设置成功，返回0失败
uint8_t jy62_setBaud(uint16_t baudRate)
{
    uint8_t flag = 1;
    if(baudRate == 115200)
        HAL_UART_Transmit_DMA(jy62_huart, baud_115200_command, 3); 
    else if(baudRate == 9600)  
        HAL_UART_Transmit_DMA(jy62_huart, baud_9600_command, 3); 
    else
        flag = 0;
    return flag;
}


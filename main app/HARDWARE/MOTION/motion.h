#ifndef __MOTION_H
#define __MOTION_H
#include "sys.h"

#define car_d 10
#define delta_t 0.01
/*
//车身参数
typedef struct __car_t{
    float d;

}car_t;
*/

/*
typedef struct __pisotion_t{
    float x;
    float y;
    float theta;
}position_t;
*/

//状态变量
typedef struct __state_t{
    float x, y, theta;//position and the direction of the car
    int v_left;//左轮速度 velocity of the left wheel
    int v_right;
    float R;//轨迹半径 Radius of motion trajectory
    float v;//车身速度 velocity of the car
    float w;// angular velocity of the car
}state_t;

state_t next, past, now;//两个时刻的状态，要做全局变量；尽量取简短的单词
//void forward();
//void reverse();
void state_update(void);
#endif


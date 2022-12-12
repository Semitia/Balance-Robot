#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define C1 PBin(4)
#define C2 PBin(3)
#define C3 PAin(5)
#define C4 PAin(4)

#define SPEED_Y 40 //俯仰(前后)最大设定速度
#define SPEED_Z 100//偏航(左右)最大设定速度 
#define ACCX_FIX 100
#define ACCY_FIX -200
#define ACCZ_FIX 16000
#define PI 3.14159265
void EXTI9_5_IRQHandler(void);
int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right,int target);
int Turn_UP(int gyro_Z, int RC);
void Tracking(void);
void print(void);
void data_receive(void);
void kalman(void);
#endif

#include "motion.h"

void forward()
{

	return;
}

void reverse()
{

	return;
}

void state_update(void)
{
	float d_theta, d_x, d_y;

	mpu_dmp_get_data(&(now.pitch),&(now.roll),&(now.yaw));										 //===得到欧拉角（姿态角）的数据
	MPU_Get_Gyroscope(&(now.gyrox),&(now.gyroy),&(now.gyroz));
	now.v_left  =  Read_Encoder(2);
	now.v_right = -Read_Encoder(3);

	now.v = (float) (now.v_left + now.v_right)/2;
	now.w = (float) (now.v_right - now.v_left)/car_d;
	now.R = (float) (now.v) / (now.w);
	
	d_theta = past.v*delta_t/past.R;
	d_x = past.R*(1-cos(d_theta));
	d_y = past.R*sin(d_theta);
	now.x = past.x + d_x*sin(past.theta) + d_y*cos(past.theta);
	now.y = past.y - d_x*cos(past.theta) + d_y*sin(past.theta);
	now.theta = past.theta + d_theta;

	past = now;
	return;
}

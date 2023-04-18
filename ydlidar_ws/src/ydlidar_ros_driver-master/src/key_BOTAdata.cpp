/**
 * @file anya_key.cpp
 * @author Semitia
 * @brief 实时监听键盘信息并依此串口发送运动控制指令
 * @version 0.1
 * @date 2023-01-22
 * @copyright Copyright (c) 2023
 */
#include "ros/ros.h"
#include "ros/time.h"
#include "ydlidar_ros_driver/serial_head.h"
#include "geometry_msgs/WrenchStamped.h"
#include <chrono>
#include <termio.h>
#include <stdio.h>

#define N 5

//using TimePoint = std::chrono::system_clock::time_point;

ros:: Timer cnt_timer;
ros::Time stamps[N];
geometry_msgs::Wrench wrench[N];
double time_point[N]={0};
double force_x[N],force_y[N],force_z[N];
double torque_x[N],torque_y[N],torque_z[N];

int scanKeyboard()
{
 
	int in;
 
	struct termios new_settings;
	struct termios stored_settings;
    //设置终端参数
	tcgetattr(0,&stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0,&stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&new_settings);
	in = getchar();
	tcsetattr(0,TCSANOW,&stored_settings);
 
	return in;//返回的是字符对应的ascii码
 
}

void div_dif(double *x, double *y,double **F)
{
    for(int i=0;i<N;i++)
    {   F[i][0] = y[i]; 
        //printf("%.0f, ",F[i][0]);
    }
    //printf("\r\n");
    for(int i=1;i<N;i++)
    {
        for(int j=0;j<N-i;j++)
        {
            F[j][i] = (F[j][i-1] - F[j+1][i-1])/(x[j] - x[j+i]);
            //printf("F[%.0f...%.0f]:%.2f, ",x[j],x[j+i],F[j][i]);
        }
        //printf("\r\n");
    }
}

double multi(double *x, double est_x, int n)
{
    double ans=1;
    for(int i=0;i<n;i++) ans *= est_x - x[i];
    return ans;
}

double newton_interpolation(double *x, double *y, double est_x)
{
    double ans=0;
    double F[N][N];//F[i][j]：从xi开始的j阶差商
    for(int i=0; i<N; i++)
    {
        ans += F[0][i]*multi(x,est_x,i);
    }
    return ans;
}

 void timer_callback(const ros::TimerEvent&)
{
    ROS_INFO("new loop");
    int key=scanKeyboard();
    switch(key)
    {
        case 'g':
        {
            //TimePoint current = std::chrono::system_clock::now();
            geometry_msgs::Wrench estimate;
            double current = ros::Time::now().toSec();
            estimate.force.x = newton_interpolation(time_point,force_x,current);
            estimate.force.y = newton_interpolation(time_point,force_y,current);
            estimate.force.z = newton_interpolation(time_point,force_z,current);
            estimate.torque.x = newton_interpolation(time_point,torque_x,current);
            estimate.torque.y = newton_interpolation(time_point,torque_y,current);
            estimate.torque.z = newton_interpolation(time_point,torque_z,current);
            break;
        }
        case 't':
        {
            ROS_INFO("test");
        }
        default :
        {
            
            break;
        }
    }
    return;
}

void data_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    //update data
    for(int i=0; i<N-1; i++) 
    {
        wrench[i] = wrench[i+1];
        stamps[i] = stamps[i+1];
    }
    stamps[N-1] = msg->header.stamp;
    wrench[N-1] = msg->wrench;
    for(int i=0; i<N; i++) 
    {
        time_point[i] = stamps[i].toSec() - stamps[0].toSec();
        force_x[i] = wrench[i].force.x;
        force_y[i] = wrench[i].force.y;
        force_z[i] = wrench[i].force.z;
        torque_x[i] = wrench[i].torque.x;
        torque_y[i] = wrench[i].torque.y;
        torque_z[i] = wrench[i].torque.z;
    }
    return;
}

//测试函数
int main(int argc, char *argv[])
{
	setlocale(LC_ALL,"");
    ros::init(argc,argv,"BOTA_data_receiver");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::WrenchStamped>("wrench",10,data_callback);
    cnt_timer = nh.createTimer(ros::Duration(0.5),timer_callback);//0.5 second
    cnt_timer.start();
    ros::spin();  
	return 0;
}
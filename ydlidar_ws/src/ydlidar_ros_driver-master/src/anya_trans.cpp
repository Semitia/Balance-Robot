/**
 * @file anya_trans.cpp
 * @author Semitia
 * @brief 将move_base的cmd_vel转换成串口运动指令
 * @version 0.1
 * @date 2023-01-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "ros/ros.h"
#include "ydlidar_ros_driver/serial_head.h"
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include <stdio.h>

#define WARN_MSG 1
#define SPD_MSG  2
#define POS_MSG  3 //position
#define PARA_MSG 4
#define DES_MSG  5 //destination
#define ACK_MSG  6
#define INFO_MSG 7//stm32's INFO

Set_Serial init;
ros:: Subscriber sub;

void swrite(char *buf, int txt, int start)
{
    int i;
	if(txt<0) {buf[start]='-';txt*=-1;}
	else 			{buf[start]='+';}
	start++;
	for(i=0;i<4;i++)
	{
		int f=(int)(pow(10,3-i));
		buf[start+i]=txt/f + 48;
		txt %= f;
	}
	//buf[++start]=0;
	return;
}

const int stop=1, mov_for=2, turn_le=3, turn_ri=4, wrong=5, mov_bac=6,for_bac=1,turn=2;
/**
 * @brief send SPD_MSG
 * @param move_cmd 
 */
void write_msg(int type, int move_cmd)
{
    //ROS_INFO("DEBUG:write msg");
    switch(move_cmd)
    {
        case stop:
        {
            ROS_INFO("STOP");
            std::string msg= "2" + std::to_string(for_bac) + "+0000\r\n";
            init.SendMsgs(msg);
            break;
        }
        case mov_for:
        {
            ROS_INFO("Move Forward");
            std::string msg= "2" + std::to_string(for_bac) + "+0500\r\n";
            init.SendMsgs(msg);
            break;
        }
        case mov_bac:
        {
            ROS_INFO("Move Back");
            std::string msg= "2" + std::to_string(for_bac) + "-0500\r\n";
            init.SendMsgs(msg);
            break;
        }
        case turn_le:
        {
            ROS_INFO("Turn Left");
            std::string msg= "2" + std::to_string(turn) + "+0100\r\n";
            init.SendMsgs(msg);    
            break;
        }
        case turn_ri:
        {
            ROS_INFO("Turn Right");
            std::string msg= "2" + std::to_string(turn) + "-0100\r\n";
            init.SendMsgs(msg);             
            break;
        }
        case wrong:
        {
            ROS_INFO("move command wrong");
            break;
        }
        default :
        {
            ROS_INFO("msg_type:%d",move_cmd);
            break;
        }
    }
    return;
}

void callback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("Received a /cmd_vel message!");
	ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);

}

int main(int argc, char *argv[])
{
	setlocale(LC_ALL,"");
    ros::init(argc,argv,"cmd_vel_listener");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    init.SerialInit("/dev/ttyUSB0");
    sub = nh.subscribe("cmd_vel",1000,callback);
    ros::spin();  
	return 0;
}

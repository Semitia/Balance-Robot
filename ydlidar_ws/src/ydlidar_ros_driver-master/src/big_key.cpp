/**
 * @file anya_key.cpp
 * @author Semitia
 * @brief 实时监听键盘信息并依此串口发送运动控制指令
 * @version 0.1
 * @date 2023-01-22
 * @copyright Copyright (c) 2023
 */
#include "ros/ros.h"
#include "ydlidar_ros_driver/serial_head.h"
#include <termio.h>
#include <stdio.h>

#define WARN_MSG 1
#define SPD_MSG  2
#define POS_MSG  3 //position
#define PARA_MSG 4
#define DES_MSG  5 //destination
#define ACK_MSG  6
#define INFO_MSG 7 //stm32's INFO

Set_Serial init;
ros:: Timer cnt_timer;
ros:: Subscriber usart_listener;

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
            std::string msg= "21+0000\r\n";
            init.SendMsgs(msg);
            break;
        }
        case mov_for:
        {
            ROS_INFO("Move Forward");
            std::string msg= "21+0800\r\n";
            init.SendMsgs(msg);
            break;
        }
        case mov_bac:
        {
            ROS_INFO("Move Back");
            std::string msg= "22+0800\r\n";
            init.SendMsgs(msg);
            break;
        }
        case turn_le:
        {
            ROS_INFO("Turn Left");
            std::string msg= "23+0800\r\n";
            init.SendMsgs(msg);    
            break;
        }
        case turn_ri:
        {
            ROS_INFO("Turn Right");
            std::string msg= "24+0800\r\n";
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

 void timer_callback(const ros::TimerEvent&)
{
    ROS_INFO("new loop");
    int key=scanKeyboard();
    std:: string got = init.SerialRead(); 
    int len=got.length();
    if(len) ROS_INFO("%s",got.c_str());
    switch(key)
    {
        case 'w':
        {
            write_msg(SPD_MSG,mov_for);
            break;
        }
        case 'a':
        {
            write_msg(SPD_MSG,turn_le);
            break;
        }
        case 's':
        {
            write_msg(SPD_MSG,mov_bac);
            break;
        }
        case 'd':
        {
            write_msg(SPD_MSG,turn_ri);
            break;
        }
        default :
        {
            write_msg(SPD_MSG,stop);
            break;
        }
    }
    return;
}

//测试函数
int main(int argc, char *argv[])
{
	setlocale(LC_ALL,"");
    ros::init(argc,argv,"Keyboard_listener");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    init.SerialInit("/dev/ttyUSB0");
    cnt_timer = nh.createTimer(ros::Duration(0.5),timer_callback);//0.5 second
    cnt_timer.start();
    ros::spin();  
	return 0;
}
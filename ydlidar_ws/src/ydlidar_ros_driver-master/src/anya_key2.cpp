/**
 * @file anya_key2.cpp
 * @author Semitia
 * @brief 车载PC订阅地面站的键盘信息并转换成串口运动指令
 * @version 0.1
 * @date 2023-01-22
 * @copyright Copyright (c) 2023
 */
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ydlidar_ros_driver/serial_head.h"

#define WARN_MSG 1
#define SPD_MSG  2
#define POS_MSG  3 //position
#define PARA_MSG 4
#define DES_MSG  5 //destination
#define ACK_MSG  6
#define INFO_MSG 7//stm32's INFO

Set_Serial init;
ros:: Timer cnt_timer;


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

void callback(const std_msgs::Int32::ConstPtr& key)
{
    ROS_INFO("new loop");
    std:: string got = init.SerialRead(); 
    int len=got.length();
    if(len) ROS_INFO("%s",got.c_str());
    switch(key->data)
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


int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"keyCMD_listener");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    init.SerialInit("/dev/ttyUSB0");
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe("keyboard",10,callback);

    ros::spin();    
    return 0;
}
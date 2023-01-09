#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
//#include "../include/serial_try/serial_head.h"
//#include "serial_try/msgs_stm32.h"
#include <limits>       // std::numeric_limits
#include <string>


enum direction {back, back_right, right, front_right, front, front_left, left, back_left};
unsigned char warn = 0;
float warn_log[8];

/*
Set_Serial init;
ros:: Timer cnt_timer;
ros:: Subscriber usart_listener;
ros:: Publisher pub;

void timer_callback(const ros::TimerEvent&)
{
    std:: string some_msgs ;
    some_msgs = "120001010\r\n";
    init.SendMsgs(some_msgs);

    std:: string data_got,data_got1;
    data_got = init.SerialRead(); 
    ROS_INFO("%s",data_got.c_str());

//  serial_try::msgs_stm32 data;
//    data.number=1;
//    pub.publish(data);
    return;
}
*/
void doScan(const sensor_msgs::LaserScan::ConstPtr& msg_s){
    //ROS_INFO("我听见:%s",msg_p->data.c_str());
    //ROS_INFO("我听见:%s",(*msg_p).data.c_str());
    //ROS_INFO("get!\r\n");
    int n,n_group;
    warn = 0;
    n = (int)(msg_s->angle_max - msg_s->angle_min)/(msg_s->angle_increment);
    n_group = n/8;
    for(int i=0; i<8; i++)
    {
        float min_range = msg_s->range_max;
        for(int j = n_group*(i); j <= n_group*(i+1); j++)
        {
            if(msg_s->ranges[j] == 0) {continue;}
            min_range = min_range > msg_s->ranges[j]?msg_s->ranges[j]:min_range;
        }
        if(min_range<1) {warn |= (0x01<<i);} 
        warn_log[i] = min_range;
    }
    /*
    if(warn & (0x01<<back))        {ROS_INFO("warn: back - %.2f\r\n",warn_log[back]);}
    if(warn & (0x01<<back_left))   {ROS_INFO("warn: back_left - %.2f\r\n",warn_log[back_left]);}
    if(warn & (0x01<<left))        {ROS_INFO("warn: left - %.2f\r\n",warn_log[left]);}
    if(warn & (0x01<<front_left))  {ROS_INFO("warn: front_left - %.2f\r\n",warn_log[front_left]);}
    if(warn & (0x01<<front))       {ROS_INFO("warn: front - %.2f\r\n",warn_log[front]);}
    if(warn & (0x01<<front_right)) {ROS_INFO("warn: front_right - %.2f\r\n",warn_log[front_right]);}
    if(warn & (0x01<<right))       {ROS_INFO("warn: right - %.2f\r\n",warn_log[right]);}
    if(warn & (0x01<<back_right))  {ROS_INFO("warn: back_right - %.2f\r\n",warn_log[back_right]);}
    */
    return;
}

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"anya_sub");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan",10,doScan);

    /*
    init.SerialInit("/dev/ttyUSB0");
    cnt_timer = nh.createTimer(ros::Duration(2),timer_callback);//2 seconds
    //nh.createtimer() is just a function whose return_type is a Timer
    pub = nh.advertise<serial_try::msgs_stm32>("msgs_stm32",10);
    cnt_timer.start();
    */
    //5.ros::spin();
    ros::spin();    
    return 0;
}
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
#include <limits>       // std::numeric_limits

void doScan(const sensor_msgs::LaserScan::ConstPtr& msg_s){
    //ROS_INFO("我听见:%s",msg_p->data.c_str());
    // ROS_INFO("我听见:%s",(*msg_p).data.c_str());
    ROS_INFO("get!\r\n");
}

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"anya");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan",10,doScan);

    //4.回调函数中处理 person

    //5.ros::spin();
    ros::spin();    
    return 0;
}
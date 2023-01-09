#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
#include <limits>       // std::numeric_limits

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"anya_pub");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

    //创建发布的数据
    sensor_msgs::LaserScan heart;
    heart.angle_min = -3.14159274101;
    heart.angle_max =  3.14159274101;
    heart.angle_increment = 0.02284794673323;
    heart.range_min = 0;
    heart.range_max = 64;
    
    //设置频率
    ros::Rate rate(1);
    //循环
    while(ros::ok())
    {
        scan_pub.publish(heart);
        rate.sleep();
        ros::spinOnce;
    }
    return 0;
}
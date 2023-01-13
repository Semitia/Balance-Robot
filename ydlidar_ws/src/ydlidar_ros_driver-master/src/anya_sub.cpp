#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
#include "ydlidar_ros_driver/serial_head.h"
#include <limits>       // std::numeric_limits
#include <string>
#include <math.h>

#define WARN_MSG 1
#define SPD_MSG  2
#define POS_MSG  3 //position
#define PARA_MSG 4
#define DES_MSG  5 //destination
#define WARN_RANGE 2//产生warn的距离阈值
#define PI 3.1315926

typedef struct __data_t{
    int x,y;
    int F,G,H;
}data_t;

typedef struct __unit_t{
    data_t data;
    unit_t *father;
    unit_t *last;
    unit_t *next;
}unit_t;

enum direction {back, back_right, right, front_right, front, front_left, left, back_left};
const int dir_angle[8] = {PI, -3*PI/4, -PI/2, -PI/4, 0, PI/4, PI/2, 3*PI/4};
unsigned char warn = 0;
float warn_log[8]; //生成警告的最短range
bool map[100][100];//地图，分辨率为1分米，范围即为10mx10m
int pos_x,pos_y,des_x,des_y;//车子坐标格子，目的地格子
float angle;//车身朝向

Set_Serial init;
ros:: Timer cnt_timer;
ros:: Subscriber usart_listener;
ros:: Publisher pub;

int tr(char t)
{ 
    return t-48; 
}

/**
 * @brief 将字符串从start位开始的num位数转换为float数。
 * @param s 字符串
 * @param start 起始位
 * @param num 位数
 * @param p 首个数乘 10^p
 * @return float 
 */
float tr_s(std:: string s, int start, int num, int p)
{
    float ans=0;
    while(num>0)
    {
        ans+= tr(s[start++])*pow(10,p--);
        num--;
    }
    return ans;
}

/**
 * @brief 向下取整并转换为int型
 * float t transport to the "map[t][t]""
 * @param t 
 * @return int 
 */
int tr_m(float t)
{ 
    return floor(t); 
}

unit_t *o_list;
unit_t *c_list;

void insert(unit_t *list, data_t new_data, unit_t *fa)
{
    unit_t *p;//插入的节点
    p = (unit_t*)malloc(sizeof(unit_t));
    p->data = new_data;
    p->father = fa;
    p->next = list->next;
    p->next->last = p;
    list->next = p;
    p->last = list;
    return;
}

void A_star_init()
{
    data_t data;
    o_list = (unit_t*)malloc(sizeof(unit_t));
    o_list->father = NULL;
    o_list->last = NULL;
    o_list->next = NULL;
    data.F = 0;
    data.G = 0;
    data.H = 0;
    data.x = pos_x;
    data.y = pos_y;
    insert(o_list,data,NULL);
    
    unit_t *c_head,*c_end,*c_normal;
    c_head = (unit_t*)malloc(sizeof(unit_t));
    c_end = c_head;
    c_end->next = NULL;
    c_head->father = NULL;


}

/**
 * @brief give the move command
 * 1:stop
 * 2:move forward
 * 3:turn left
 * 4:turn right
 * 5:
 * @return move command
 */
int A_star()
{

}

void timer_callback(const ros::TimerEvent&)
{
    std:: string warn_msgs;
    warn=123;
    warn_msgs = "0" + std::to_string(warn);
    init.SendMsgs(warn_msgs);

    std:: string got;
    got = init.SerialRead(); 

    /*data_receive*/
    switch (tr(got[0]))
    {
        case DES_MSG://先设定传来的单位是分米，如100.1dm，精确度为1cm。为了方便初始化，这里面也包含小车坐标。
        {
            des_x = floor(tr_s(got,1,4,2) );
            des_y = floor(tr_s(got,5,4,2) );
            pos_x = tr_m(tr_s(got, 9,4,2) );
            pos_y = tr_m(tr_s(got,13,4,2) );
            A_star_init();
            //ACK
            break;
        }
        case POS_MSG://角度信息，360.0度
        {
            float tem_x = tr_s(got,1,4,2);
            float tem_y = tr_s(got,5,4,2);
            angle = tr_s(got,9,4,2);
            pos_x = floor(tem_x);
            pos_y = floor(tem_y);
            for(int i=0; i<8; i++)
            {
                float dis = WARN_RANGE;//扫到的距离
                if((warn>>i)&0x01) {dis = warn_log[i];}
                float wall_x = tem_x + dis*cos(angle + dir_angle[i]);
                float wall_y = tem_y + dis*sin(angle + dir_angle[i]);
                map[tr_m(wall_x)][tr_m(wall_y)] = (warn>>i)&0x01;
            }

            break;
        }
        default :
        {
            ROS_INFO("USART WRONG!");
            break;
        }
    }
    ROS_INFO("%s",got.c_str());


    return;
}



void doScan(const sensor_msgs::LaserScan::ConstPtr& msg_s){
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
        if(min_range<WARN_RANGE) {warn |= (0x01<<i);} 
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

    
    init.SerialInit("/dev/ttyUSB0");
    cnt_timer = nh.createTimer(ros::Duration(1),timer_callback);//2 seconds
    //nh.createtimer() is just a function whose return_type is a Timer
    cnt_timer.start();
    
    ros::spin();    
    return 0;
}
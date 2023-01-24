/**
 * @file anya_sub.cpp
 * @author Semitia
 * @brief 解析/scan话题，与stm32建立串口通讯并指导避障。
 *        使用A_Star算法发送运动控制指令 或 直接发送简化的障碍物情况
 * @version 0.1
 * @date 2023-01-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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
#define ACK_MSG  6
#define INFO_MSG 7//stm32's INFO

#define WARN_RANGE 0.25//产生warn的距离阈值
#define ANG_RANGE  0.15//前进时允许的角度偏差
#define PI 3.1315926
#define BES_VALUE 10//value of going to the unit beside car
#define COR_VALUE 14//value of going to the unit on the corner
#define COR_LOCK  1 //如果是1，那么右边是墙的时候无法到达右上角，其他三个角同理。
#define MODE 1 //1:A* mode; 2:PS2 mode

typedef struct __data_t{
    int x,y;
    int F,G,H;
}data_t;

typedef struct __unit_t unit_t;
typedef struct __unit_t{
    data_t data;
    unit_t *father;
    unit_t *last;
    unit_t *next;
}unit_t;

enum direction {back, back_right, right, front_right, front, front_left, left, back_left};
const double dir_angle[8] = {PI, -3*PI/4, -PI/2, -PI/4, 0, PI/4, PI/2, 3*PI/4};
unsigned char warn = 0;
float warn_log[8]; //生成警告的最短range
bool map[200][200];//地图，分辨率为1分米，范围即为10mx10m：0可走，1为障碍物
bool open_flag[200][200],close_flag[200][200];//record if the unit has been add to any list.
int pos_x=0,pos_y=0,des_x=0,des_y=0;//车子坐标格子，目的地格子
float _pos_x=0,_pos_y=0,_des_x=0,_des_y=0;//非栅格化的真实坐标
float angle=0;//车身朝向

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
    //ROS_INFO("DEBUG:tr_s");
    float ans=0;
    short negative=1;
    if(s[start++] == '-') {negative=-1;}
    num--;
    while(num>0)
    {
        int number = tr(s[start++]);
        ans+= number*pow(10,p--);
        //ROS_INFO("number:%d, ans:%.3f",number,ans);
        num--;
    }
    ans*=negative;
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

/**
 * @brief read arr[x][y]
 * 
 * @param arr 
 * @param x 
 * @param y 
 * @return true 
 * @return false 
 */
bool r(bool arr[][200],int x,int y)
{
    if(x<-99 || y<-99 || x>99 || y>99) {return 0;}
    if(x<0) x+=200;
    if(y<0) y+=200;
    return arr[x][y];
}

/**
 * @brief write arr[x][y]
 * 
 * @param arr 
 * @param x 
 * @param y 
 * @param value 
 * @return true 
 * @return false 
 */
bool w(bool arr[][200],int x,int y,bool value)
{
    if(x<-99 || y<-99 || x>99 || y>99) {return 0;}
    if(x<0) x+=200;
    if(y<0) y+=200;
    arr[x][y] = value;
    return 0;
}

void print_map(int tox, int toy)
{
    for(int y=30;y>=-30;y--)
    {
        for(int x=-30;x<=30;x++)
        {
            if(x==tox && y==toy) {printf("%d",3);}
            else if(x==0 && y==0) printf("%d",2);
            else printf("%d",r(map,x,y));
        }
        printf("\r\n");
    }
    printf("\r\n");
    return;
}


unit_t *o_list;
unit_t *c_list;
void insert_node(unit_t *list, data_t new_data, unit_t *fa)
{
    if(list == o_list) 
    {
        ROS_INFO("open_list");
        w(open_flag,new_data.x,new_data.y,true);
    }
    else 
    {
        ROS_INFO("close_list");
        w(close_flag,new_data.x,new_data.y,true);
    }
    ROS_INFO("DEBUG:insert node:%d,%d",new_data.x,new_data.y);
    unit_t *p;//插入的节点
    p = (unit_t*)malloc(sizeof(unit_t));
    p->data = new_data;
    p->father = fa;
    p->next = list->next;
    if(p->next != NULL) {p->next->last = p;}
    list->next = p;
    p->last = list;
    //ROS_INFO("DEBUG:OUT insert node()");
    return;
}

void delete_node(unit_t *p)
{
    ROS_INFO("DEBUG:delete node:%d,%d",p->data.x,p->data.y);
    if(p->last!=NULL) p->last->next = p->next;
    if(p->next!=NULL) p->next->last = p->last;
    //free(p);
}

bool A_init_flag=0;
void A_star_init()
{
    if(A_init_flag) {return;}
    ROS_INFO("DEBUG:A_Star_init()");
    data_t data;
    o_list = (unit_t*)malloc(sizeof(unit_t));
    o_list->father = NULL;
    o_list->last = NULL;
    o_list->next = NULL;
    data.G = 0;
    data.H = (abs(des_x-pos_x) + abs(des_y-pos_y))*20;
    data.F = data.H;
    data.x = pos_x;
    data.y = pos_y;
    insert_node(o_list,data,NULL);
    //open_flag[pos_x][pos_y] = true;

    c_list = (unit_t*)malloc(sizeof(unit_t));
    c_list->father = NULL;
    c_list->last = NULL;
    c_list->next = NULL;
    ROS_INFO("DEBUG:OUT A_Star_init");
    A_init_flag=1;
    return;
}

//enum mov_cmd {stop=1, mov_for, turn_le, turn_ri, wrong};
const int stop=1, mov_for=2, turn_le=3, turn_ri=4, wrong=5, mov_bac=6,for_bac=1,turn=2;
/**
 * @brief 
 * @param from 起点
 * @param to 目的地
 * @return 运动模式
 */
int moveto(unit_t *to)//暂时感觉格式还不太优美，有待优化
{
    ROS_INFO("DEBUG:%.2f,%.2fmove to:%d,%d",_pos_x,_pos_y,to->data.x,to->data.y);
    print_map(to->data.x,to->data.y);
    float to_ang;
    if(to->data.x == _pos_x)
    {
        if(to->data.y > _pos_y) {to_ang = PI/2;}
        else if(to->data.y < _pos_y) {to_ang = -PI/2;}
        else {return stop;}
    }
    else {to_ang = atan2((to->data.y-_pos_y),(to->data.x-_pos_x));}
    
    ROS_INFO("target angel: %.2f",to_ang);
    if(abs(to_ang - angle)<ANG_RANGE) {return mov_for;}
    else if(to_ang < angle) 
    {
        float left_ang = to_ang + PI*2 - angle;
        float right_ang = angle - to_ang;
        if(left_ang < right_ang) {return turn_le;}
        else {return turn_ri;}
    }
    else 
    {
        float left_ang = to_ang - angle;
        float right_ang = angle + PI*2 - to_ang;
        if(left_ang < right_ang) {return turn_le;}
        else {return turn_ri;}
    }
    return wrong;
}

/**
 * @brief //到了终点所在格子，进行更精细的运动
 * 
 * @return int 
 */
int moveto(void)
{
    ROS_INFO("DEBUG:%.2f,%2.fmove to:%.2f,%.2f",_pos_x,_pos_y,_des_x,_des_y);
    float to_ang;
    if(_des_x==_pos_x)
    {
        if(_des_y==_pos_y) {return stop;}
        else if(_des_y>_pos_y) {to_ang=PI/2;}
        else {to_ang=-PI/2;}
    }
    else { to_ang = atan((_des_y-_pos_y)/(_des_x-_pos_x)); }
    if(abs(to_ang - angle)<ANG_RANGE) {return mov_for;}
    else if(to_ang < angle) {return turn_ri;}
    else {return turn_le;}
    return wrong;
}

/**
 * @brief 判断搜索的点是不是在地图之外
 * 
 * @param u 
 * @return true 
 * @return false 
 */
bool outof_range(data_t u)
{
    if(u.x>100 || u.x<0) {return 1;}
    if(u.y>100 || u.y<0) {return 1;}
    return 0;
}

//需要与enum那里的顺序保持一致
const int around[8][2] = {{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};
unit_t *tar_unit;//车子正在前往的地图单位:target unit
/**
 * @brief run A* alogrithm and give move command
 * 1:stop
 * 2:move forward
 * 3:turn left
 * 4:turn right
 * 5:
 * @return move command
 */
void A_star()
{
    if(!A_init_flag) {return;}
    if(tar_unit!=NULL)
    {
        float tar_dis = abs(tar_unit->data.x-_pos_x) + abs(tar_unit->data.y - _pos_y);
        if(tar_dis>4) {ROS_INFO("Wait");return;} 
    }
    ROS_INFO("DEBUG:A_Star");
    if(tar_unit!=NULL && tar_unit->data.x == des_x && tar_unit->data.y == des_y)
    {ROS_INFO("Search Completed");return;}

    unit_t *p_min = o_list->next;//找F值最小的节点
    ROS_INFO("Searching......");
    for(unit_t *p = o_list->next; p!=NULL; p = p->next)
    {
        ROS_INFO("F(%d,%d):%d",p->data.x,p->data.y,p->data.F);
        if(p_min->data.F > p->data.F) {p_min = p;}
    }
    ROS_INFO("center point: %d,%d",p_min->data.x,p_min->data.y);
    for(int i=0; i<8; i++)
    {
        data_t find;
        find.x = p_min->data.x + around[i][0];
        find.y = p_min->data.y + around[i][1];

        if(outof_range(find)) {continue;}///////////////////////////////////////////////////////
        if(r(map,find.x,find.y) || r(close_flag,find.x,find.y)) {continue;}
        if(COR_LOCK && i%2) //对角不能随便走
        {
            if(r(map,find.x,p_min->data.y) || r(map,p_min->data.x,find.y)) 
            {continue;}//
        }

        if(i%2) {find.G = p_min->data.G+COR_VALUE;}//奇数对应斜角
        else    {find.G = p_min->data.G+BES_VALUE;}
        find.H = (abs(des_x-find.x) + abs(des_y-find.y))*20; 
        find.F = find.G + find.H;
        ROS_INFO("find a new unit");
        if(r(open_flag,find.x,find.y)) 
        {
            //寻找这个节点
            unit_t *find_p;
            for(unit_t *p=o_list->next; p!=NULL; p=p->next)
            {
                if((p->data.x==find.x) && p->data.y==find.y) 
                {
                    find_p=p;
                    break;
                }
            }
            if(find.G < find_p->data.G)
            {
                find_p->data.G = find.G;
                find_p->data.F = find.G + find.H;
                find_p->father = p_min;
            }
        }
        else
        {
            insert_node(o_list,find,p_min);
        }
    }
    delete_node(p_min);
    insert_node(c_list,p_min->data,p_min->father);

    /*前往p_min的位置*/
    tar_unit = p_min;
    return;
}

int move_ctl()
{
    if(!A_init_flag) {return stop;}

    if(tar_unit!=NULL && (pos_x!=tar_unit->data.x || pos_y!=tar_unit->data.y))
    { return moveto(tar_unit); }

    if((pos_x==des_x) && (pos_y==des_y))//到了终点所在格子，进行更精细的运动
    {return moveto();}

    ROS_WARN("Unknown situation");
    return stop;
}

/**
 * @brief send WARN_MSG
 */
void write_msg(int type)
{
    //ROS_INFO("DEBUG:write msg");
    switch (type)
    {
        case WARN_MSG:
        {
            ROS_INFO("Send Warn=%u",warn);
            std::string msg= "1" + std::to_string(warn);
            init.SendMsgs(msg);
            break;            
        }

        case ACK_MSG:
        {
            ROS_INFO("Send ACK");
            std::string msg= "6\r\n";
            init.SendMsgs(msg);
            break;
        }
    }
    return;
}

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
            break;
        }
        case turn_le:
        {
            ROS_INFO("Turn Left");
            std::string msg= "2" + std::to_string(turn) + "+0050\r\n";
            init.SendMsgs(msg);    
            break;
        }
        case turn_ri:
        {
            ROS_INFO("Turn Right");
            std::string msg= "2" + std::to_string(turn) + "-0050\r\n";
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
    std:: string warn_msgs;
    //warn_msgs = "0" + std::to_string(warn);
    //init.SendMsgs(warn_msgs);

    std:: string got;
    got = init.SerialRead(); 
    int len=got.length();
    /*data_receive*/
    
    switch (tr(got[0]))
    {
        case DES_MSG://先设定传来的单位是分米，如100.1dm，精确度为1cm。为了方便初始化，这里面也包含小车坐标。
        {
            _des_x = tr_s(got,1,5,1);
            _des_y = tr_s(got,6,5,1);
            _pos_x = tr_s(got, 11,5,1);
            _pos_y = tr_s(got,16,5,1);
            angle = tr_s(got,21,5,0);
            des_x = tr_m(_des_x);
            des_y = tr_m(_des_y);
            pos_x = tr_m(_pos_x);
            pos_y = tr_m(_pos_y);            
            ROS_INFO("got DES_MSG:_desx:%.2f,_desy:%.2f,desx:%d,desy:%d",_des_x,_des_y,des_x,des_y);
            ROS_INFO("_posx:%.2f,_posy:%.2f,posx:%dposy:%d,angle:%.2f",_pos_x,_pos_y,pos_x,pos_y,angle);
            A_star_init();
            write_msg(ACK_MSG);
            break;
        }
        case POS_MSG://坐标和角度信息，360.0度
        {
            _pos_x = tr_s(got,1,5,1);
            _pos_y = tr_s(got,6,5,1);
            angle = tr_s(got,11,5,0);
            pos_x = tr_m(_pos_x);
            pos_y = tr_m(_pos_y);
            //ROS_INFO("got POS_MSG:_posx:%.2f,_posy:%.2f,posx:%dposy:%d,angle:%.2f",_pos_x,_pos_y,pos_x,pos_y,angle);
            ROS_INFO("got POS_MSG:posx:%d,posy:%d,angle:%.2f",pos_x,pos_y,angle);

            for(int i=0; i<8; i++)
            {
                float dis = WARN_RANGE*10;//扫到的距离
                if((warn>>i)&0x01) {dis = warn_log[i]*10;}
                float wall_x = _pos_x + dis*cos(angle + dir_angle[i]);
                float wall_y = _pos_y + dis*sin(angle + dir_angle[i]);
                if(wall_x>0&&wall_x<100&&wall_y>0&&wall_y<100)
                w(map,tr_m(wall_x),tr_m(wall_y),(warn>>i)&0x01);
            }
            break;
        }
        case INFO_MSG:
        {
            ROS_INFO("%s",got.c_str());
            break;
        }
        default :
        {
            //ROS_WARN("USART WRONG!");
            break;
        }
    }
    
    if(len) ROS_INFO("%s",got.c_str());
    

    switch (MODE)
    {
        case 1:
        {
            A_star();
            int move_cmd = move_ctl();
            write_msg(SPD_MSG,move_cmd);
            break;
        }
    }
    

    return;
}

void doScan(const sensor_msgs::LaserScan::ConstPtr& msg_s)
{

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

    for(int i=0; i<n; i++)
    {
        float map_ang = angle - PI + i*msg_s->angle_increment;
        float dis = msg_s->ranges[i]*10;
        if(dis == 0) {continue;}
        float wall_x = _pos_x + dis*cos(map_ang);
        float wall_y = _pos_y + dis*sin(map_ang);
        //ROS_INFO("i:%d,angle:%.3f,dis:%.2f,wallx:%d,wally:%d",i,map_ang,dis,tr_m(wall_x),tr_m(wall_y));
        //if(dis<msg_s->range_max)
        /**/
        for(float inc_dis=0.9; inc_dis<dis; inc_dis+=0.9)
        {
            float line_x = _pos_x + inc_dis*cos(map_ang);
            float line_y = _pos_y + inc_dis*sin(map_ang);
            //if(line_x>0&&line_x<100&&line_y>0&&line_Fy<100)
            w(map,tr_m(line_x),tr_m(line_y),0);
        }
        
        int wall_mx=tr_m(wall_x),wall_my=tr_m(wall_y);
        w(map,wall_mx-1,wall_my,1);
        w(map,wall_mx-2,wall_my,1);
        w(map,wall_mx+1,wall_my,1);
        w(map,wall_mx+2,wall_my,1);
        w(map,wall_mx-1,wall_my-1,1);
        w(map,wall_mx-1,wall_my+1,1);
        w(map,wall_mx+1,wall_my-1,1);
        w(map,wall_mx+1,wall_my+1,1);
        w(map,wall_mx,wall_my-1,1);
        w(map,wall_mx,wall_my+1,1);
        w(map,wall_mx,wall_my-2,1);
        w(map,wall_mx,wall_my+2,1);
    }
    //print_map();

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
    cnt_timer = nh.createTimer(ros::Duration(0.2),timer_callback);//2 seconds
    //nh.createtimer() is just a function whose return_type is a Timer
    cnt_timer.start();
    ros::spin();    
    return 0;
}

#include "../include/sp25/sp25.h"

int main(int argc, char** argv)
{
    //初始化ROS节点，节点名称请使用与ROS包名相同
    ros::init(argc, argv, "sp25");
    // //定义类对象指针
    ros::NodeHandle nh;
    double frq = 50;
    Sp25::Sp25Driver *exe = new Sp25::Sp25Driver(nh, frq);
    //调用对象run函数，外部直接调用的类应当包含至少一个run函数
    exe->run();
    //关闭ROS节点
    ros::shutdown();
    return 0;
}
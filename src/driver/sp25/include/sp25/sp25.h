/**
 * @file    sp25.h
 * @author  何永旗
 * @date    2023-4-15
 *
 * @brief   sp25类头文件，声明类
*/

#ifndef _SP25_H
#define _SP25_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>
#include <sensor/Mwradar.h>
#include "stdio.h"

using namespace std;

namespace Sp25{
    //常量定义
    const uint32_t      DATA_SIZE               =14;
    const uint32_t      BUFFER_SIZE             =DATA_SIZE*3 + DATA_SIZE - 1;
    const string        SERIAL_NAME             ="/dev/ttyUSB0";     //="/dev/ttyUSB0";
    const uint32_t      SERIAL_BAUD             =115200;
    const uint32_t      SERIAL_WAIT             =10;
    const string        FRAME_ID                ="SP25";

    class Sp25Driver
    {
        private:
            ros::NodeHandle rosNode;
            double rosLoopRate;
            // uint8_t* insData;                   //有效数据
            uint8_t* insBuffer;                    //缓冲区
            serial::Serial Sp25Serial;             //接收串口
            vector<uint8_t> insData;               //存放输入数据
            sensor::Mwradar msgMwradar;            //毫米波雷达消息

            ros::Publisher pubMwradar;             //
        public:
            //构造函数
            Sp25Driver(ros::NodeHandle &nh, double frq);
            //析构函数
            virtual ~Sp25Driver();
            //主函数
            void run();
        private:
            //打开串口
            bool OpenSerial();
            //关闭串口
		    bool CloseSerial();
		    //发布消息
            void PublishMsg(const ros::TimerEvent &e);
            //检查数据并发送
            bool DataCalculate();
    };
}
#endif
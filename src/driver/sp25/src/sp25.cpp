#include "../include/sp25/sp25.h"
/**
 * @file    sp25.h
 * @author  何永旗
 * @date    2023-4-15
 *
 * @brief   sp25类头文件，声明类
 */
namespace Sp25
{
    Sp25Driver::Sp25Driver(ros::NodeHandle &nh, double frq)
    {
        rosNode = nh;
        rosLoopRate = frq;

        insBuffer = new uint8_t[BUFFER_SIZE];

        // 定义发布器
        pubMwradar = nh.advertise<sensor::Mwradar>("millimeter/wave/radar", 1);
    }

    Sp25Driver::~Sp25Driver()
    {
        delete[] insBuffer;
    }

    void Sp25Driver::run()
    {
        // 打开串口
        if (!OpenSerial())
        {
            return;
        }
        
        ros::Timer t = rosNode.createTimer(ros::Duration(1 / rosLoopRate), &Sp25Driver::PublishMsg, this);
        
        ros::spin();

        // 关闭串口
        CloseSerial();
    }

    bool Sp25Driver::OpenSerial()
    {
        // 设置串口属性
        Sp25Serial.setPort(SERIAL_NAME);
        // 设置串口波特率
        Sp25Serial.setBaudrate(SERIAL_BAUD);
        // 设置超时
        serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_WAIT);
        Sp25Serial.setTimeout(to);
        // 尝试打开串口
        try
        {
            Sp25Serial.open();
        }
        catch (serial::IOException &e)
        {
            // 报错误
            ROS_ERROR_STREAM("open serial failed!!! exam serial!!!!");
            return false;
        }

        // 再次检查串口打开
        if (!Sp25Serial.isOpen())
        {
            ROS_ERROR_STREAM("serial open failed!!!");
            return false;
        }

        ROS_INFO("serial is opened!!!!");
        return true;
    }

    bool Sp25Driver::CloseSerial()
    {
        Sp25Serial.close();
        // 检测串口是否关闭
        if (Sp25Serial.isOpen())
        {
            ROS_INFO("serial close failed!!!!!!");
            return false;
        }

        ROS_INFO("serial is closed!!!!!");
        return true;
    }

    void Sp25Driver::PublishMsg(const ros::TimerEvent &e)
    {
        // 检查是否收到数据，未收到数据则退出
        if (!Sp25Serial.available())
        {
            return;
        }
        // 读串口函数，参数一为缓冲器，参数二为读取缓冲器中的字节数
        Sp25Serial.read(insBuffer, BUFFER_SIZE);
        Sp25Serial.flushInput();
        for(int i = 0; i <  BUFFER_SIZE; i++)
        {
            if(insBuffer[i] == 0xAA) //数据为帧头
            {
                if(insData.size() == 0)                       //栈内无数据，插入
                    insData.push_back(insBuffer[i]); 
                else if(insData.size() == 1 && insData[0] == 0xAA) //栈内只有一个数据并且为帧头，插入
                    insData.push_back(insBuffer[i]);
                else                             //其他情况清空栈，插入
                {                                           
                    insData.clear();  
                    insData.push_back(insBuffer[i]);
                }   
            }
            else if(insBuffer[i] == 0x55) //数据为帧尾
            {
                if(insData.size() > 0)  //栈内有数据，插入，并检查发送
                {
                    insData.push_back(insBuffer[i]);

                }
                else                    //栈内无数据，舍弃
                {}
            }
            else                    //数据为内容
            {
                if(insData.size() > 0)   //栈中有数据
                    insData.push_back(insBuffer[i]);
                else                    //栈内无数据，舍弃
                {}       
            }
            if(insData.size() == DATA_SIZE)
            {
                if(DataCalculate())
                    pubMwradar.publish(msgMwradar);  
                insData.clear();
            }
        }
    }
    bool Sp25Driver::DataCalculate()
    {
        if(insData.size() == DATA_SIZE) //先检查数据长度
        {
            if (insData[0] == 0xAA && insData[1] == 0xAA && insData[DATA_SIZE - 1] == 0x55 && insData[DATA_SIZE - 2] == 0x55) //检查帧头，帧尾
            {
                if(insData[2] == 0x0C && insData[3] == 0x07) //检查MessageID
                {
                    msgMwradar.index = insData[4];                                                                // 目标ID                                                     
                    msgMwradar.rcs = (double)(insData[5] * 0.5 - 50);                                             // 出厂测试使用参数
                    msgMwradar.range = (double)((short int)(insData[6] * 256 + insData[7]) * 0.01);               // 目标距离 单位为m
                    msgMwradar.rollcount = (insData[9] & 0xC0);                                                   // 计数位
                    msgMwradar.verl = (double)((short int)((insData[9] & 0x07) * 256 + insData[10]) * 0.05 - 35); // 目标速度，单位m/s
                    msgMwradar.snr = insData[11] - 127;                                                           // 出厂测试保留值，不做输出
                    return true;
                }
                else 
                    return false;
            }
            else 
                return false;
        }
        else
            return false;
    }
}

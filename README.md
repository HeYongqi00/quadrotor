## <center>无人机避障
通过SP25毫米波雷达读取障碍物的速度的距离信息。
#### 1.读取串口中可能会遇到的问题
程序运行之后如果不能打开串口，执行命令
`sudo gedit /etc/udev/rules.d/70-ttyusb.rules`

在打开的文件(每个电脑中的文件可能不一样，采用tab键补全就知道自己的文件名字是什么了)中添加
`KERNEL=="ttyUSB[0-9]*", MODE="0666"`

查看自己电脑上的可用串口命令
`ls /dev/tty*`


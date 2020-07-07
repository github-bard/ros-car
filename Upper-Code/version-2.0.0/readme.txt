#检查确认底盘的串口号，如若不是ttyUSB0则在base_controller.cpp文件中修改串口号
  ls -l /dev |grep ttyUSB
#查看串口属性
  stty -F /dev/ttyUSB0
#修改波特率
  stty -F /dev/ttyUSB0  115200
#赋予串口权限
  sudo chmod 777 /dev/ttyUSB0
#查看topic消息
  rostopic echo /cmd_vel
  rostopic echo /Joy
#运行程序，可实现手柄控制
  rosrun joy_ctrl joy_c
  rosrun joy joy_node
  rosrun base_controller base_controller



# 基于ROS的红绿灯识别仿真
## 介绍
这是工科创课程的大作业，基于已有的在gazebo中实现阿克曼小车巡线的场景，增添了识别红绿灯的功能，使用了传统视觉的方式完成。  
**使用环境：ubuntu20.04 + ros1-noetic**
## 快速使用
1. 将项目克隆到本地
   打开终端，输入`git clone https://github.com/joeynie/traffic_light_detection.git`
2. 编译并配置环境变量 `catkin_make && source ./devel/setup.bash`
3. 加载gazebo环境（可能要等一段时间）
`roslaunch raceworld_master raceworld2.launch`
5. 运行红绿灯检测程序  
打开另一个终端
```
source ./devel/setup.bash
rosrun raceworld_master camera
```
6. 运行巡线程序  
打开另一个终端
```
source ./devel/setup.bash
rosrun raceworld_master running_car2.py
```

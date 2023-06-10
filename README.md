
简体中文 | [English](./README.en.md) 

<h1 align="center">NLink Parser ROS Package</h1>

<div align="center">

![Logo](./image/nooploop.png)

本ROS包用于支持Nooploop产品，采用C++编写，只需根据对应产品手册，通过我们的上位机NAssistant配置好模块，确定设备已正常工作后，运行本ROS包中产品对应的节点即可获取产品协议一一对应的ROS消息，为您省去处理数据通信及解析的时间

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://dev.azure.com/ant-design/ant-design-pro/_build/latest?definitionId=1?branchName=master) ![Test Status](https://img.shields.io/badge/test-passing-brightgreen)

</div>

支持的产品

- [LinkTrack](https://www.nooploop.com/) 是一款基于UWB技术的多功能系统，集定位、分布式测距、授时及通信功能一体化，典型二维定位精度±10cm，数据更新频率高达200Hz，基站容量多达120个，标签容量多达200个，全部节点无线自动组网，基站坐标一键标定，便捷部署。

  ![linktrack](./image/linktrack_clip_720p_4fps.gif)

- [LinkTrack AOA](http://www.nooploop.com/linktrack-aoa) 是一款基于UWB的高精度跟随系统，典型测距精度高达±5cm，典型测向精度高达±5°，刷新频率高达200Hz，集测距、测向、授时、通信为一体。

- [TOFSense](https://www.nooploop.com/tofsense) 是一款基于TOF(飞行时间)技术的激光测距传感器。测距范围 1cm~5m，距离分辨率 1mm，数据更新频率 10Hz，可调FOV，最大视场角 27°，支持多传感器级联输出。


## Table of Contents

- [Table of Contents](#table-of-contents)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Building](#building)
  - [Unit Tests](#unit-tests)
- [Products Usage](#products-usage)
  - [LinkTrack](#linktrack)
  - [LinkTrack AOA](#linktrack-aoa)
  - [TOFSense](#tofsense)
  - [TOFSense-M](#tofsense-m)
  - [IOT](#iot)
- [How to Subscribe Our Topic](#how-to-subscribe-our-topic)
- [Submodule](#submodule)
  - [nlink\_unpack](#nlink_unpack)
  - [protocol\_extracter](#protocol_extracter)
- [License](#license)
- [Bugs \& Feature Requests](#bugs--feature-requests)
- [FAQ](#faq)
  

## Getting Started

### Prerequisites

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
  
    运行本程序需要ROS支持，如还未安装请先安装ROS。

- [Serial Library](https://github.com/nooploop-dev/serial.git)

    本程序和硬件设备之间采用串口通信，请先点击安装该串口通信库，注意，如果是第一次使用串口设备，通常需要开启串口操作权限，详情参考[Fix serial port permission denied errors on Linux](https://websistent.com/fix-serial-port-permission-denied-errors-linux/)

### Building

按如下步骤克隆代码并编译

    cd catkin_workspace/src
    git clone --recursive https://github.com/nooploop-dev/nlink_parser.git 
    cd ../
    catkin_make
    source devel/setup.bash

注意，每次打开新命令行窗口都需要执行 `source devel/setup.bash` 重新获取该ROS工作空间环境变量.

### Unit Tests

执行单元测试

    roscore
    catkin_make run_tests

## Products Usage

### LinkTrack

运行

    roslaunch nlink_parser linktrack.launch

参数
   - **`port_name`** 设备串行端口名称，默认值: `/dev/ttyUSB0`.
   - **`baud_rate`** 设备波特率，默认值: `921600`.
  
如需要RVIZ显示可执行消息转换器

    roslaunch nlink_parser linktrack_rviz.launch

订阅的话题

* **`/nlink_linktrack_data_transmission`** ([std_msgs::String])

	你可以通过对该话题发布消息，将数据发送给LinkTrack节点，进而利用数传功能

发布的话题

  - **`/nlink_linktrack_anchorframe0`** ([nlink_parser::LinktrackAnchorframe0])
  - **`/nlink_linktrack_tagframe0`** ([nlink_parser::LinktrackTagframe0])
  - **`/nlink_linktrack_nodeframe0`** ([nlink_parser::LinktrackNodeframe0])
  - **`/nlink_linktrack_nodeframe1`** ([nlink_parser::LinktrackNodeframe1])
  - **`/nlink_linktrack_nodeframe2`** ([nlink_parser::LinktrackNodeframe2])
  - **`/nlink_linktrack_nodeframe3`** ([nlink_parser::LinktrackNodeframe3])
  - **`/nlink_linktrack_nodeframe4`** ([nlink_parser::LinktrackNodeframe4])
  - **`/nlink_linktrack_nodeframe5`** ([nlink_parser::LinktrackNodeframe5])
  - **`/nlink_linktrack_nodeframe6`** ([nlink_parser::LinktrackNodeframe6])

    如果收到来自其他节点的数传数据，则 `/nlink_linktrack_nodeframe0` 话题将会发布消息
    
    其他话题为定位数据话题，当收到协议帧数据，将自动在对应话题上发布消息，协议类型需要在上位机NAssistant上进行配置

### LinkTrack AOA

运行

    roslaunch nlink_parser linktrack_aoa.launch

参数
   - **`port_name`** 设备串行端口名称，默认值: `/dev/ttyUSB0`.
   - **`baud_rate`** 设备波特率，默认值: `921600`.
  
订阅的话题

* **`/nlink_linktrack_data_transmission`** ([std_msgs::String])

	你可以通过对该话题发布消息，将数据发送给LinkTrack节点，进而利用数传功能

发布的话题

  - **`/nlink_linktrack_nodeframe0`** ([nlink_parser::LinktrackNodeframe0])
  - **`/nlink_linktrack_aoa_nodeframe0`** ([nlink_parser::LinktrackAoaNodeframe0])


### TOFSense

运行

    roslaunch nlink_parser tofsense.launch

参数
   - **`port_name`** 设备串行端口名称，默认值: `/dev/ttyUSB0`.
   - **`baud_rate`** 设备波特率，默认值: `921600`.
   - **`inquire_mode`** 用于多TOFSense级联情况，程序将自动查询并输出多节点测距结果，默认值：`false`.

发布的话题

  - **`/nlink_tofsense_cascade`** ([nlink_parser::TofsenseCascade]) 
  - **`/nlink_tofsense_frame0`** ([nlink_parser::TofsenseFrame0])

### TOFSense-M

运行

    roslaunch nlink_parser tofsensem.launch

参数
   - **`port_name`** 设备串行端口名称，默认值: `/dev/ttyUSB0`.
   - **`baud_rate`** 设备波特率，默认值: `921600`.

发布的话题

  - **`/nlink_tofsensem_frame0`** ([nlink_parser::TofsenseMFrame0])


### IOT

运行

    roslaunch nlink_parser iot.launch

参数
   - **`port_name`** 设备串行端口名称，默认值: `/dev/ttyUSB0`.
   - **`baud_rate`** 设备波特率，默认值: `921600`.

发布的话题

  - **`/nlink_iot_frame0`** ([nlink_parser::IotFrame0])



## How to Subscribe Our Topic

参考 [nlink_example](https://github.com/nooploop-dev/nlink_example)，在您的ROS包中订阅我们的话题

## Submodule

### [nlink_unpack](https://github.com/nooploop-dev/nlink_unpack)

用于支持Nooploop产品，如[LinkTrack](https://www.nooploop.com/)，[LinkTrack AOA](http://www.nooploop.com/linktrack-aoa)，[TOFSense](https://www.nooploop.com/tofsense)，纯C语言编写，用户可用于构建协议解析代码

### [protocol_extracter](https://github.com/nooploop-dev/protocol_extracter) 

是一个通用的数据协议帧提取器，对协议实现及协议检测进行了解耦，用户只需传入数据，处理协议回调函数即可


## License

源码基于 [BSD 3-Clause license](LICENSE) 许可发布


## Bugs & Feature Requests

问题反馈及功能建议请使用 [Issue Tracker](https://github.com/nooploop-dev/nlink_parser/issues).

## FAQ
- 运行 `rostopic echo ...` 查看话题数据，提示"ERROR: Cannot load message class for ... Are your messages built
  
  如果已经编译通过，请运行  `source {$ros_workspace}/devel/setup.bash` 以添加环境变量，其中{$ros_workspace}表示当前ros包所在的工作空间目录。

- 运行节点提示 "error while loading shared libraries: libserial.so: Cannot open shared object file: No such file or directory"
  
  如果确定已经按照前面链接安装了串口库，编译也正常，仅运行时提示找不到库文件，尝试

    1. 运行 `sudo gedit /etc/ld.so.conf.d/libc.conf`
    2. 如果没有 `/usr/local/lib` ,则进行添加
    3. 保存文件，然后执行 `sudo ldconfig`
    4. 重启电脑

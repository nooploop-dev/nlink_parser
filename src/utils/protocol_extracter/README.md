
简体中文 | [English](./README.en.md) 

<h1 align="center">Protocol Extracter</h1>

<div align="center">

![Logo](http://ftp.nooploop.com/media/image/nooploop.png)

Protocol Extracter 是一个通用的数据协议帧提取器，对协议实现及协议检测进行了解耦，用户只需传入数据，处理协议回调函数即可

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://dev.azure.com/ant-design/ant-design-pro/_build/latest?definitionId=1?branchName=master) ![Test Status](https://img.shields.io/badge/test-passing-brightgreen)


</div>

## Usage

- NProtocolBase 作为通用协议基类，由用户根据其协议进行继承，填写帧头，实现校验方式等
- NProtocolExtracter 作为协议提取器，管理协议，输入数据，对数据进行自动拼接，并提取协议帧

使用示例参考[example.c](./example.cpp)文件

## License

源码基于[BSD 3-Clause license](LICENSE)发布



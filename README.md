![Logo of the project](http://ftp.nooploop.com/media/image/nooploop.png)
# nlink_parser ROS Package

## Overview

This package is developed for [LinkTrack](https://www.nooploop.com/) and [TOFSense](https://www.nooploop.com/tofsense) written in C++, you can get the data from device you need directly without additional process.

Refer to [NLink document](http://ftp.nooploop.com/software/products/uwb/doc/NLink_V1.3.pdf)

**Keywords:** ros, nlink parser, serial

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Samuel Hsu<br />
Affiliation: [Nooploop](https://www.nooploop.com/)<br />
Maintainer: Samuel Hsu, nooploop.studio@gmail.com**

The nlink_parser package has been tested under [ROS] Kinetic and Ubuntu 16.04. 


## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- ros-serial

		sudo apt-get install ros-kinetic-serial

    Please make sure you have the permission to read or write to serial device, refer to [Fix serial port permission denied errors on Linux](https://websistent.com/fix-serial-port-permission-denied-errors-linux/)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone --recursive https://github.com/NooploopStudio/nlink_parser.git 
	cd ../
	catkin_make
    source devel/setup.bash


### Unit Tests

Run the unit tests with

	catkin_make run_tests


## Usage

Run LinkTrack node with

	roslaunch nlink_parser linktrack.launch

Param
   - **`port_name`** serial port name of the device. Default: `/dev/ttyUSB0`.
   - **`baud_rate`** baud rate of the device. Default: `921600`.

Run TOFSense node with

	roslaunch nlink_parser tofsense.launch

Param
   - **`port_name`** serial port name of the device. Default: `/dev/ttyUSB0`.
   - **`baud_rate`** baud rate of the device. Default: `921600`.
   - **`inquire_mode`** if true, automatic query nodes and output them together. Default: `true`.

## Nodes

### linktrack

Read and unpack serial data from device.When a protocol passes verification, topic of it will be registered and published. 


#### Subscribed Topics

* **`/nlink_linktrack_data_transmission`** ([std_msgs::String])

	You can write data to the device by sending a message to this topic


#### Published Topics

  - **`/nlink_linktrack_anchorframe0`** ([nlink_parser::LinktrackAnchorframe0])
  - **`/nlink_linktrack_tagframe0`** ([nlink_parser::LinktrackTagframe0])
  - **`/nlink_linktrack_nodeframe0`** ([nlink_parser::LinktrackNodeframe0])
  - **`/nlink_linktrack_nodeframe1`** ([nlink_parser::LinktrackNodeframe1])
  - **`/nlink_linktrack_nodeframe2`** ([nlink_parser::LinktrackNodeframe2])
  - **`/nlink_linktrack_nodeframe3`** ([nlink_parser::LinktrackNodeframe3])



### tofsense

same as linktrack. 

#### Published Topics

  - **`/nlink_tofsense_cascade`** ([nlink_parser::TofsenseCascade]) 
  - **`/nlink_tofsense_frame0`** ([nlink_parser::TofsenseFrame0])


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/NooploopStudio/nlink_parser/issues).


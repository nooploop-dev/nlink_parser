#include <ros/ros.h>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include <chrono>
#include <thread>

int main(int argc, char **argv) {
  ros::init(argc, argv, "iot_parser");
  ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);

  NProtocolExtracter extracter;
  iot::Init init(&extracter);
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      extracter.AddNewData(str_received);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}

#include <ros/ros.h>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

#include <iomanip>
#include <iostream>

void printHexData(const std::string &data) {
  if (!data.empty()) {
    std::cout << "data received: ";
    for (int i = 0; i < data.size(); ++i) {
      std::cout << std::hex << std::setfill('0') << std::setw(2)
                << int(uint8_t(data.at(i))) << " ";
    }
    std::cout << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_parser");
  ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);
  NProtocolExtracter protocol_extraction;
  linktrack::Init init(&protocol_extraction, &serial);
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      // printHexData(str_received);
      protocol_extraction.AddNewData(str_received);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}

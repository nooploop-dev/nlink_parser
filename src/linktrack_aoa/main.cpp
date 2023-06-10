#include <ros/ros.h>

#include "init.h"
#include "init_serial.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_aoa");
  ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);
  NProtocolExtracter protocol_extraction;
  linktrack_aoa::Init aoaInit(&protocol_extraction, &serial);
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      protocol_extraction.AddNewData(str_received);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}

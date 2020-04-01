#include "init.h"
#include <init_serial.h>
#include <nlink/nframe_extraction.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tofsense_parser");
  ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);

  NFrameExtraction frameExtraction;
  TOFSense::Init init(&frameExtraction, &serial);

  while (ros::ok()) {
    auto availableBytes = serial.available();
    std::string strReceived;
    if (availableBytes) {
      serial.read(strReceived, availableBytes);
      frameExtraction.unpackData(strReceived);
    }
    ros::spinOnce();
  }
  return EXIT_SUCCESS;
}

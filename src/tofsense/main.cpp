#include "init.h"
#include <init_serial.h>
#include <nlink/nframe_extraction.h>
#include <ros/ros.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "tofsense_parser");
  serial::Serial serial;
  initSerial(&serial);

  NFrameExtraction frameExtraction;
  TOFSense::Init tofsenseInit(&frameExtraction, &serial);

  ros::Rate loopRate(1000);
  while (ros::ok()) {

    auto availableBytes = serial.available();
    std::string strReceived;
    if (availableBytes) {
      serial.read(strReceived, availableBytes);
      frameExtraction.unpackData(strReceived);
    }
    loopRate.sleep();
  }
  return EXIT_SUCCESS;
}

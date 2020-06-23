#include <init_serial.h>
#include <nlink/nframe_extraction.h>
#include <ros/ros.h>

#include "init.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linktrack_aoa");
  ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);
  NFrameExtraction frameExtraction;
  LinkTrackAoa::Init aoaInit(&frameExtraction, &serial);
  ros::Rate loopRate(1000);
  while (ros::ok())
  {
    auto availableBytes = serial.available();
    std::string strReceived;
    if (availableBytes)
    {
      serial.read(strReceived, availableBytes);
      frameExtraction.unpackData(strReceived);
    }
    ros::spinOnce();
    loopRate.sleep();
  }
  return EXIT_SUCCESS;
}

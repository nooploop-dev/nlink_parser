#include <ros/ros.h>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tofsense_parser");
  ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);

  NProtocolExtracter extracter;
  tofsense::Init init(&extracter, &serial);

  while (ros::ok())
  {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes)
    {
      serial.read(str_received, available_bytes);
      extracter.AddNewData(str_received);
    }
    ros::spinOnce();
  }
  return EXIT_SUCCESS;
}

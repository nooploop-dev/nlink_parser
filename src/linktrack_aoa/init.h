#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe0.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <unordered_map>

#include "protocol_extracter/nprotocol_extracter.h"

namespace linktrack_aoa
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial);

  private:
    void initDataTransmission();
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction);
    std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
    ros::NodeHandle nh_;
    ros::Subscriber dt_sub_;
  };
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H

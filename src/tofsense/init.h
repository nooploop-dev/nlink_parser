#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

#include <nlink_parser/TofsenseCascade.h>
#include <nlink_parser/TofsenseFrame0.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <map>
#include <unordered_map>

#include "protocol_extracter/nprotocol_extracter.h"

namespace tofsense {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction,
                serial::Serial *serial);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);

  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;

  std::map<int, nlink_parser::TofsenseFrame0> frame0_map_;

  serial::Serial *serial_;

  const int frequency_ = 10;
  bool is_inquire_mode_ = true;

  ros::NodeHandle nh_;
  ros::Timer timer_scan_;
  ros::Timer timer_read_;
  uint8_t node_index_ = 0;
};

} // namespace tofsense
#endif // TOFSENSEINIT_H

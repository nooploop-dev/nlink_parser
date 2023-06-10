#ifndef TOFSENSEMINIT_H
#define TOFSENSEMINIT_H

#include "protocol_extracter/nprotocol_extracter.h"
#include <nlink_parser/TofsenseMFrame0.h>
#include <ros/ros.h>
#include <unordered_map>

namespace tofsensem {
class Init {
public:
  explicit Init(NProtocolExtracter *protocol_extraction);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);
  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
  ros::NodeHandle nh_;
};

} // namespace tofsensem
#endif // TOFSENSEMINIT_H

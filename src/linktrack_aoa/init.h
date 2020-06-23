#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <nlink/nframe_extraction.h>
#include <nlink/nprotocolbase.h>
#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe0.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <unordered_map>

class NFrameExtraction;
namespace LinkTrackAoa
{
class Init
{
public:
  static nlink_parser::LinktrackNodeframe0 msgNodeFrame0Data_;
  static nlink_parser::LinktrackAoaNodeframe0 msgAoaNodeFrame0Data_;

  explicit Init(NFrameExtraction *frameExtraction, serial::Serial *serial);

private:
  void initDataTransmission();
  void initNodeFrame0(NFrameExtraction *frameExtraction);
  void initAoaNodeFrame0(NFrameExtraction *frameExtraction);
  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
  ros::NodeHandle nodeHandle_;
  ros::Subscriber dtSub_;
};
}  // namespace LinkTrackAoa

#endif  // LINKTRACKAOAINIT_H

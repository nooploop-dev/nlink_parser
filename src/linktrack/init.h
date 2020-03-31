#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include <nlink/nframe_extraction.h>
#include <nlink/nprotocolbase.h>
#include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe1.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <unordered_map>

class NFrameExtraction;
namespace LinkTrack {

class Init {
public:
  static nlink_parser::LinktrackAnchorframe0 msgAnchorFrame0Data_;
  static nlink_parser::LinktrackTagframe0 msgTagFrame0Data_;
  static nlink_parser::LinktrackNodeframe0 msgNodeFrame0Data_;
  static nlink_parser::LinktrackNodeframe1 msgNodeFrame1Data_;
  static nlink_parser::LinktrackNodeframe2 msgNodeFrame2Data_;
  static nlink_parser::LinktrackNodeframe3 msgNodeFrame3Data_;

  explicit Init(NFrameExtraction *frameExtraction, serial::Serial *serial);

private:
  void initDataTransmission();
  void initAnchorFrame0(NFrameExtraction *frameExtraction);
  void initTagFrame0(NFrameExtraction *frameExtraction);
  void initNodeFrame0(NFrameExtraction *frameExtraction);
  void initNodeFrame1(NFrameExtraction *frameExtraction);
  void initNodeFrame2(NFrameExtraction *frameExtraction);
  void initNodeFrame3(NFrameExtraction *frameExtraction);

  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
  ros::NodeHandle nodeHandle_;
  ros::Subscriber dtSub_;
};
} // namespace LinkTrack

#endif // LINKTRACKINIT_H

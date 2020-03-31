#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

#include <init_serial.h>
#include <map>
#include <nlink/nframe_extraction.h>
#include <nlink/tofsense_protocols.h>
#include <nlink_parser/TofsenseCascade.h>
#include <nlink_parser/TofsenseFrame0.h>
#include <ros/ros.h>
#include <unordered_map>

namespace TOFSense {

class Init {
public:
  explicit Init(NFrameExtraction *frameExtraction, serial::Serial *serial);
  //  ~Init();

private:
  void initFrame0(NFrameExtraction *frameExtraction);

  //  void test(NFrameExtraction *frameExtraction);

  NTS_ProtocolFrame0 *protocolFrame0_;

  std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;

  std::map<int, nlink_parser::TofsenseFrame0> frame0Map_;

  serial::Serial *serial_;

  /** 根据实际情况自行配置参数 **/
  const int frequency_ = 10;  //查询频率,最大10Hz
  bool isInquireMode_ = true; //主动查询模式
  /** 根据实际情况自行配置参数 **/

  ros::NodeHandle nh;
  ros::Timer timerScan_;
  ros::Timer timerRead_;
  uint8_t nodeIndex_ = 0;
};

} // namespace TOFSense
#endif // TOFSENSEINIT_H

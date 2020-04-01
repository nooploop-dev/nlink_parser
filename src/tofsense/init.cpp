#include "init.h"
#include <nlink/nlink_unpack/nlink_utils.h>
#include <nutils.h>

namespace TOFSense {
#pragma pack(push, 1)
struct {
  char header[2]{0x57, 0x10};
  uint8_t reserved0[2]{0xff, 0xff};
  uint8_t id{};
  uint8_t reserved1[2]{0xff, 0xff};
  uint8_t checkSum{};
} readFrame_;
#pragma pack(pop)

Init::Init(NFrameExtraction *frameExtraction, serial::Serial *serial)
    : serial_(serial) {

  isInquireMode_ =
      serial_ ? ros::param::param<bool>("~inquire_mode", true) : false;

  initFrame0(frameExtraction);
}

/*
// called per ms
void Init::updateStatus() {
  if (isInquireMode_) {
    static int ms = 0;
    if (ms % 100 == 0) {
      frame0Map_.clear();
      nodeIndex_ = 0;
    }
    if (ms % 6 == 0) {
      if (nodeIndex_ < 8) {
        readFrame_.id = nodeIndex_;
        auto data = reinterpret_cast<uint8_t *>(&readFrame_);
        updateCheckSum(data, sizeof(readFrame_));
        serial_->write(data, sizeof(readFrame_));
      } else if (nodeIndex_ == 8) {
        if (!frame0Map_.empty()) {

          nlink_parser::TofsenseCascade msgCascade;
          for (const auto &msg : frame0Map_) {
            msgCascade.nodes.push_back(msg.second);
          }
          publishers_.at(protocolFrame0_).publish(msgCascade);
        }
      }
    }
    ++nodeIndex_;
    ++ms;
  }
}
*/
void Init::initFrame0(NFrameExtraction *frameExtraction) {
  protocolFrame0_ = new NTS_ProtocolFrame0;
  frameExtraction->addProtocol(protocolFrame0_);
  protocolFrame0_->setDataUseHandle([=] {
    if (!publishers_[protocolFrame0_]) {
      ros::NodeHandle nodeHandle;
      if (isInquireMode_) {
        auto topic = "nlink_tofsense_cascade";
        publishers_[protocolFrame0_] =
            nodeHandle.advertise<nlink_parser::TofsenseCascade>(topic, 50);
        topicadvertisedTip(topic);
      } else {
        auto topic = "nlink_tofsense_frame0";
        publishers_[protocolFrame0_] =
            nodeHandle.advertise<nlink_parser::TofsenseFrame0>(topic, 50);
        topicadvertisedTip(topic);
      }
    }
    const auto &data = ntsFrame0_.data;
    nlink_parser::TofsenseFrame0 frame0MsgData;

    frame0MsgData.id = data.id;
    frame0MsgData.systemTime = data.systemTime;
    frame0MsgData.distance = data.distance;
    frame0MsgData.distanceStatus = data.distanceStatus;
    frame0MsgData.signalStrength = data.signalStrength;

    if (isInquireMode_) {
      frame0Map_[data.id] = frame0MsgData;
    } else {
      publishers_.at(protocolFrame0_).publish(frame0MsgData);
    }
  });

  if (isInquireMode_) {

    timerScan_ = nh.createTimer(ros::Duration(1.0 / frequency_),
                                [=](const ros::TimerEvent &) {
                                  frame0Map_.clear();
                                  nodeIndex_ = 0;
                                  timerRead_.start();
                                },
                                false, true);
    timerRead_ = nh.createTimer(
        ros::Duration(0.006),
        [=](const ros::TimerEvent &) {
          if (nodeIndex_ >= 8) {
            if (!frame0Map_.empty()) {
              nlink_parser::TofsenseCascade msgCascade;
              for (const auto &msg : frame0Map_) {
                msgCascade.nodes.push_back(msg.second);
              }
              publishers_.at(protocolFrame0_).publish(msgCascade);
            }
            timerRead_.stop();
          } else {
            readFrame_.id = nodeIndex_;
            auto data = reinterpret_cast<uint8_t *>(&readFrame_);
            updateCheckSum(data, sizeof(readFrame_));
            serial_->write(data, sizeof(readFrame_));
            ++nodeIndex_;
          }
        },
        false, false);
  }
}
/*
void Init::test(NFrameExtraction *frameExtraction) {
  frameExtraction->unpackData(
      NCommon::StringToHex("57 00 ff 00 9e 8f 00 00 ad 08 00 00 03 00 ff 3a"));
}
*/

} // namespace TOFSense

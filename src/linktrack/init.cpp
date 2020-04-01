#include "init.h"
#include <nlink/linktrack_protocols.h>
#include <nutils.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#define ARRAY_ASSIGN(DEST, SRC)                                                \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) {         \
    DEST[_CNT] = SRC[_CNT];                                                    \
  }

namespace LinkTrack {

nlink_parser::LinktrackAnchorframe0 Init::msgAnchorFrame0Data_;
nlink_parser::LinktrackTagframe0 Init::msgTagFrame0Data_;
nlink_parser::LinktrackNodeframe0 Init::msgNodeFrame0Data_;
nlink_parser::LinktrackNodeframe1 Init::msgNodeFrame1Data_;
nlink_parser::LinktrackNodeframe2 Init::msgNodeFrame2Data_;
nlink_parser::LinktrackNodeframe3 Init::msgNodeFrame3Data_;

static serial::Serial *serial_;

Init::Init(NFrameExtraction *frameExtraction, serial::Serial *serial) {
  serial_ = serial;
  initDataTransmission();
  initAnchorFrame0(frameExtraction);
  initTagFrame0(frameExtraction);
  initNodeFrame0(frameExtraction);
  initNodeFrame1(frameExtraction);
  initNodeFrame2(frameExtraction);
  initNodeFrame3(frameExtraction);
}

static void dtCallback(const std_msgs::String::ConstPtr &msg) {
  if (serial_)
    serial_->write(msg->data);
}

void Init::initDataTransmission() {
  dtSub_ = nodeHandle_.subscribe("nlink_linktrack_data_transmission", 1000,
                                 dtCallback);
}

void Init::initAnchorFrame0(NFrameExtraction *frameExtraction) {
  auto protocol = new NLT_ProtocolAnchorFrame0;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_anchorframe0";
      publishers_[protocol] =
          nodeHandle_.advertise<nlink_parser::LinktrackAnchorframe0>(topic,
                                                                     200);
      topicadvertisedTip(topic);
    }
    auto data = nltAnchorFrame0_.data;
    //    nlink_parser::LinktrackAnchorframe0 msgAnchorFrame0Data_;
    msgAnchorFrame0Data_.role = data.role;
    msgAnchorFrame0Data_.id = data.id;
    msgAnchorFrame0Data_.voltage = data.voltage;
    msgAnchorFrame0Data_.localTime = data.localTime;
    msgAnchorFrame0Data_.systemTime = data.systemTime;
    auto &msgTags = msgAnchorFrame0Data_.tag;
    const auto &tag = data.tag;
    msgTags.clear();
    decltype(msgAnchorFrame0Data_.tag)::value_type msgTag;
    for (size_t i = 0, icount = data.validNodeCount; i < icount; ++i) {
      msgTag.role = tag[i].role;
      msgTag.id = tag[i].id;
      ARRAY_ASSIGN(msgTag.pos, data.tag[i].pos)
      ARRAY_ASSIGN(msgTag.dis, data.tag[i].dis)
      msgTags.push_back(msgTag);
    }
    publishers_.at(protocol).publish(msgAnchorFrame0Data_);
  });
}

void Init::initTagFrame0(NFrameExtraction *frameExtraction) {
  auto protocol = new NLT_ProtocolTagFrame0;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_tagframe0";
      publishers_[protocol] =
          nodeHandle_.advertise<nlink_parser::LinktrackTagframe0>(topic, 200);
      topicadvertisedTip(topic);
    }

    const auto &data = nltTagFrame0_.data;
    auto &msgData = msgTagFrame0Data_;

    msgData.role = data.role;
    msgData.id = data.id;
    msgData.localTime = data.localTime;
    msgData.systemTime = data.systemTime;
    msgData.voltage = data.voltage;
    ARRAY_ASSIGN(msgData.pos, data.pos)
    ARRAY_ASSIGN(msgData.eop, data.eop)
    ARRAY_ASSIGN(msgData.vel, data.vel)
    ARRAY_ASSIGN(msgData.dis, data.dis)
    ARRAY_ASSIGN(msgData.imuGyro, data.imuGyro)
    ARRAY_ASSIGN(msgData.imuAcc, data.imuAcc)
    ARRAY_ASSIGN(msgData.angle, data.angle)
    ARRAY_ASSIGN(msgData.q, data.q)

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initNodeFrame0(NFrameExtraction *frameExtraction) {
  auto protocol = new NLT_ProtocolNodeFrame0;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe0";
      publishers_[protocol] =
          nodeHandle_.advertise<nlink_parser::LinktrackNodeframe0>(topic, 200);
      topicadvertisedTip(topic);
      ;
    }
    const auto &data = nltNodeFrame0_.data;
    auto &msgData = msgNodeFrame0Data_;
    auto &msgNodes = msgData.node;

    msgData.role = data.role;
    msgData.id = data.id;

    msgNodes.resize(data.validNodeCount);
    for (size_t i = 0; i < data.validNodeCount; ++i) {
      auto &msgNode = msgNodes[i];
      auto node = data.node[i];
      msgNode.id = node->id;
      msgNode.role = node->role;
      msgNode.data.resize(node->dataLength);
      memcpy(msgNode.data.data(), node->data, node->dataLength);
    }

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initNodeFrame1(NFrameExtraction *frameExtraction) {
  auto protocol = new NLT_ProtocolNodeFrame1;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe1";
      publishers_[protocol] =
          nodeHandle_.advertise<nlink_parser::LinktrackNodeframe1>(topic, 200);
      topicadvertisedTip(topic);
    }
    const auto &data = nltNodeFrame1_.data;
    auto &msgData = msgNodeFrame1Data_;
    auto &msgNodes = msgData.node;

    msgData.role = data.role;
    msgData.id = data.id;
    msgData.localTime = data.localTime;
    msgData.systemTime = data.systemTime;
    msgData.voltage = data.voltage;

    msgNodes.resize(data.validNodeCount);
    for (size_t i = 0; i < data.validNodeCount; ++i) {
      auto &msgNode = msgNodes[i];
      auto node = data.node[i];
      msgNode.id = node->id;
      msgNode.role = node->role;
      ARRAY_ASSIGN(msgNode.pos, node->pos)
    }

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initNodeFrame2(NFrameExtraction *frameExtraction) {

  auto protocol = new NLT_ProtocolNodeFrame2;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe2";
      publishers_[protocol] =
          nodeHandle_.advertise<nlink_parser::LinktrackNodeframe2>(topic, 200);
      topicadvertisedTip(topic);
    }
    const auto &data = nltNodeFrame2_.data;
    auto &msgData = msgNodeFrame2Data_;
    auto &msgNodes = msgData.node;

    msgData.role = data.role;
    msgData.id = data.id;
    msgData.localTime = data.localTime;
    msgData.systemTime = data.systemTime;
    msgData.voltage = data.voltage;
    ARRAY_ASSIGN(msgData.pos, data.pos)
    ARRAY_ASSIGN(msgData.eop, data.eop)
    ARRAY_ASSIGN(msgData.vel, data.vel)
    ARRAY_ASSIGN(msgData.imuGyro, data.imuGyro)
    ARRAY_ASSIGN(msgData.imuAcc, data.imuAcc)
    ARRAY_ASSIGN(msgData.angle, data.angle)
    ARRAY_ASSIGN(msgData.q, data.q)

    msgNodes.resize(data.validNodeCount);
    for (size_t i = 0; i < data.validNodeCount; ++i) {
      auto &msgNode = msgNodes[i];
      auto node = data.node[i];
      msgNode.id = node->id;
      msgNode.role = node->role;
      msgNode.dis = node->dis;
      msgNode.fpRssi = node->fpRssi;
      msgNode.rxRssi = node->rxRssi;
    }

    publishers_.at(protocol).publish(msgData);
  });
}

void Init::initNodeFrame3(NFrameExtraction *frameExtraction) {

  auto protocol = new NLT_ProtocolNodeFrame3;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe3";
      publishers_[protocol] =
          nodeHandle_.advertise<nlink_parser::LinktrackNodeframe3>(topic, 200);
      topicadvertisedTip(topic);
    }
    const auto &data = nltNodeFrame3_.data;
    auto &msgData = msgNodeFrame3Data_;
    auto &msgNodes = msgData.node;

    msgData.role = data.role;
    msgData.id = data.id;
    msgData.localTime = data.localTime;
    msgData.systemTime = data.systemTime;
    msgData.voltage = data.voltage;

    msgNodes.resize(data.validNodeCount);
    for (size_t i = 0; i < data.validNodeCount; ++i) {
      auto &msgNode = msgNodes[i];
      auto node = data.node[i];
      msgNode.id = node->id;
      msgNode.role = node->role;
      msgNode.dis = node->dis;
      msgNode.fpRssi = node->fpRssi;
      msgNode.rxRssi = node->rxRssi;
    }

    publishers_.at(protocol).publish(msgData);
  });
}
} // namespace LinkTrack

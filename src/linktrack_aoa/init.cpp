#include <nlink/linktrack_protocols.h>
#include <nutils.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "init.h"

namespace LinkTrackAoa
{
nlink_parser::LinktrackNodeframe0 Init::msgNodeFrame0Data_;
nlink_parser::LinktrackAoaNodeframe0 Init::msgAoaNodeFrame0Data_;

static serial::Serial *serial_;

Init::Init(NFrameExtraction *frameExtraction, serial::Serial *serial)
{
  serial_ = serial;
  initDataTransmission();
  initNodeFrame0(frameExtraction);
  initAoaNodeFrame0(frameExtraction);
}

static void dtCallback(const std_msgs::String::ConstPtr &msg)
{
  if (serial_)
    serial_->write(msg->data);
}

void Init::initDataTransmission()
{
  dtSub_ = nodeHandle_.subscribe("nlink_linktrack_data_transmission", 1000, dtCallback);
}

void Init::initNodeFrame0(NFrameExtraction *frameExtraction)
{
  auto protocol = new NLT_ProtocolNodeFrame0;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol])
    {
      auto topic = "nlink_linktrack_nodeframe0";
      publishers_[protocol] = nodeHandle_.advertise<nlink_parser::LinktrackNodeframe0>(topic, 200);
      topicadvertisedTip(topic);
      ;
    }
    const auto &data = nltNodeFrame0_.data;
    auto &msgData = msgNodeFrame0Data_;
    auto &msgNodes = msgData.node;

    msgData.role = data.role;
    msgData.id = data.id;

    msgNodes.resize(data.validNodeCount);
    for (size_t i = 0; i < data.validNodeCount; ++i)
    {
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

void Init::initAoaNodeFrame0(NFrameExtraction *frameExtraction)
{
  auto protocol = new NLTAoa_ProtocolNodeFrame0;
  frameExtraction->addProtocol(protocol);
  protocol->setDataUseHandle([=] {
    if (!publishers_[protocol])
    {
      auto topic = "nlink_linktrack_aoa_nodeframe0";
      publishers_[protocol] = nodeHandle_.advertise<nlink_parser::LinktrackAoaNodeframe0>(topic, 200);
      topicadvertisedTip(topic);
    }
    const auto &data = nltAoaNodeFrame0_.data;
    auto &msgData = msgAoaNodeFrame0Data_;
    auto &msgNodes = msgData.node;

    msgData.role = data.role;
    msgData.id = data.id;
    msgData.localTime = data.localTime;
    msgData.systemTime = data.systemTime;
    msgData.voltage = data.voltage;

    msgNodes.resize(data.validNodeCount);
    for (size_t i = 0; i < data.validNodeCount; ++i)
    {
      auto &msgNode = msgNodes[i];
      auto node = data.node[i];
      msgNode.id = node->id;
      msgNode.role = node->role;
      msgNode.dis = node->dis;
      msgNode.angle = node->angle;
      msgNode.fpRssi = node->fpRssi;
      msgNode.rxRssi = node->rxRssi;
    }

    publishers_.at(protocol).publish(msgData);
  });
}

}  // namespace LinkTrackAoa

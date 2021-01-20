#include "init.h"

#include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe1.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include <nlink_parser/LinktrackSettingframe0.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "nutils.h"
#include "protocols.h"

#define ARRAY_ASSIGN(DEST, SRC)                                        \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) { \
    DEST[_CNT] = SRC[_CNT];                                            \
  }

namespace linktrack {
nlink_parser::LinktrackAnchorframe0 g_msg_anchorframe0;
nlink_parser::LinktrackTagframe0 g_msg_tagframe0;
nlink_parser::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser::LinktrackNodeframe1 g_msg_nodeframe1;
nlink_parser::LinktrackNodeframe2 g_msg_nodeframe2;
nlink_parser::LinktrackNodeframe3 g_msg_nodeframe3;
nlink_parser::LinktrackSettingframe0 g_msg_settingframe0;

serial::Serial *serial_;

Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial) {
  serial_ = serial;
  InitDataTransmission();
  initAnchorFrame0(protocol_extraction);
  initTagFrame0(protocol_extraction);
  InitNodeFrame0(protocol_extraction);
  initNodeFrame1(protocol_extraction);
  initNodeFrame2(protocol_extraction);
  initNodeFrame3(protocol_extraction);
  initSettingFrame0(protocol_extraction);
}

static void DTCallback(const std_msgs::String::ConstPtr &msg) {
  if (serial_) serial_->write(msg->data);
}

void Init::InitDataTransmission() {
  dt_sub_ =
      nh_.subscribe("nlink_linktrack_data_transmission", 1000, DTCallback);
}

void Init::initAnchorFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolAnchorFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_anchorframe0";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackAnchorframe0>(topic, 200);
      TopicAdvertisedTip(topic);
    }
    auto data = nlt_anchorframe0_.result;
    g_msg_anchorframe0.role = data.role;
    g_msg_anchorframe0.id = data.id;
    g_msg_anchorframe0.voltage = data.voltage;
    g_msg_anchorframe0.local_time = data.local_time;
    g_msg_anchorframe0.system_time = data.system_time;
    auto &msg_nodes = g_msg_anchorframe0.nodes;
    msg_nodes.clear();
    decltype(g_msg_anchorframe0.nodes)::value_type msg_node;
    for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i) {
      auto node = data.nodes[i];
      msg_node.role = node->role;
      msg_node.id = node->id;
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
      ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr)
      msg_nodes.push_back(msg_node);
    }
    publishers_.at(protocol).publish(g_msg_anchorframe0);
  });
}

void Init::initTagFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolTagFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_tagframe0";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackTagframe0>(topic, 200);
      TopicAdvertisedTip(topic);
    }

    const auto &data = g_nlt_tagframe0.result;
    auto &msg_data = g_msg_tagframe0;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
    ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
    ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
    ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr)
    ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
    ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
    ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
    ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

    publishers_.at(protocol).publish(msg_data);
  });
}

void Init::InitNodeFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe0";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackNodeframe0>(topic, 200);
      TopicAdvertisedTip(topic);
      ;
    }
    const auto &data = g_nlt_nodeframe0.result;
    auto &msg_data = g_msg_nodeframe0;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.data.resize(node->data_length);
      memcpy(msg_node.data.data(), node->data, node->data_length);
    }

    publishers_.at(protocol).publish(msg_data);
  });
}

void Init::initNodeFrame1(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame1;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe1";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackNodeframe1>(topic, 200);
      TopicAdvertisedTip(topic);
    }
    const auto &data = g_nlt_nodeframe1.result;
    auto &msg_data = g_msg_nodeframe1;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
    }

    publishers_.at(protocol).publish(msg_data);
  });
}

void Init::initNodeFrame2(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame2;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe2";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackNodeframe2>(topic, 200);
      TopicAdvertisedTip(topic);
    }
    const auto &data = g_nlt_nodeframe2.result;
    auto &msg_data = g_msg_nodeframe2;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
    ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
    ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
    ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
    ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
    ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
    ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

    publishers_.at(protocol).publish(msg_data);
  });
}

void Init::initNodeFrame3(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame3;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_nodeframe3";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackNodeframe3>(topic, 200);
      TopicAdvertisedTip(topic);
    }
    const auto &data = g_nlt_nodeframe3.result;
    auto &msg_data = g_msg_nodeframe3;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

    publishers_.at(protocol).publish(msg_data);
  });
}

void Init::initSettingFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolSettingFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_settingframe0";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackSettingframe0>(topic, 200);
      TopicAdvertisedTip(topic);
    }
    const auto &data = g_nlt_settingframe0.result;
    auto &msg_data = g_msg_settingframe0;
    auto &msg_nodes = msg_data.nodes;

    msg_data.mix = data.mix;
    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.group = data.group;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      ARRAY_ASSIGN(msg_node.ag_3d, node->ag_3d)
    }

    publishers_.at(protocol).publish(msg_data);
  });
}
}  // namespace linktrack

#include "init.h"

#include <std_msgs/String.h>

#include "../linktrack/protocols.h"
#include "nlink_protocol.h"
#include "nlink_unpack/nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nutils.h"

class NLTAoa_ProtocolNodeFrame0 : public NLinkProtocolVLength {
public:
  NLTAoa_ProtocolNodeFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

NLTAoa_ProtocolNodeFrame0::NLTAoa_ProtocolNodeFrame0()
    : NLinkProtocolVLength(true, g_nltaoa_nodeframe0.fixed_part_size,
                           {g_nltaoa_nodeframe0.frame_header,
                            g_nltaoa_nodeframe0.function_mark}) {}

void NLTAoa_ProtocolNodeFrame0::UnpackFrameData(const uint8_t *data) {
  g_nltaoa_nodeframe0.UnpackData(data, length());
}

namespace linktrack_aoa {
nlink_parser::LinktrackNodeframe0 g_msg_nodeframe0;
nlink_parser::LinktrackAoaNodeframe0 g_msg_aoa_nodeframe0;

static serial::Serial *g_serial;

Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial) {
  g_serial = serial;
  initDataTransmission();
  initNodeFrame0(protocol_extraction);
  InitAoaNodeFrame0(protocol_extraction);
}

static void DTCallback(const std_msgs::String::ConstPtr &msg) {
  if (g_serial)
    g_serial->write(msg->data);
}

void Init::initDataTransmission() {
  dt_sub_ =
      nh_.subscribe("nlink_linktrack_data_transmission", 1000, DTCallback);
}

void Init::initNodeFrame0(NProtocolExtracter *protocol_extraction) {
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

void Init::InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLTAoa_ProtocolNodeFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      auto topic = "nlink_linktrack_aoa_nodeframe0";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::LinktrackAoaNodeframe0>(topic, 200);
      TopicAdvertisedTip(topic);
    }
    const auto &data = g_nltaoa_nodeframe0.result;
    auto &msg_data = g_msg_aoa_nodeframe0;
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
      msg_node.angle = node->angle;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

    publishers_.at(protocol).publish(msg_data);
  });
}

} // namespace linktrack_aoa

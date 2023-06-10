
#include "init.h"

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_tofsensem_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"

namespace {
class ProtocolFrame0 : public NLinkProtocolVLength {
public:
  ProtocolFrame0()
      : NLinkProtocolVLength(
            true, g_ntsm_frame0.fixed_part_size,
            {g_ntsm_frame0.frame_header, g_ntsm_frame0.function_mark}) {}

protected:
  bool UpdateLength(const uint8_t *data, size_t available_bytes) override {
    if (available_bytes < g_ntsm_frame0.fixed_part_size)
      return false;
    return set_length(tofm_frame0_size(data));
  }
  void UnpackFrameData(const uint8_t *data) override {
    g_ntsm_frame0.UnpackData(data, length());
  }
};
} // namespace

namespace tofsensem {
nlink_parser::TofsenseMFrame0 g_msg_tofmframe0;

Init::Init(NProtocolExtracter *protocol_extraction) {
  InitFrame0(protocol_extraction);
}

void Init::InitFrame0(NProtocolExtracter *protocol_extraction) {
  static auto protocol = new ProtocolFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([=] {
    if (!publishers_[protocol]) {
      ros::NodeHandle nh_;
      auto topic = "nlink_tofsensem_frame0";
      publishers_[protocol] =
          nh_.advertise<nlink_parser::TofsenseMFrame0>(topic, 50);
      TopicAdvertisedTip(topic);
    }

    const auto &data = g_ntsm_frame0;
    g_msg_tofmframe0.id = data.id;
    g_msg_tofmframe0.system_time = data.system_time;
    g_msg_tofmframe0.pixels.resize(data.pixel_count);
    for (int i = 0; i < data.pixel_count; ++i) {
      const auto &src_pixel = data.pixels[i];
      auto &pixel = g_msg_tofmframe0.pixels[i];
      pixel.dis = src_pixel.dis;
      pixel.dis_status = src_pixel.dis_status;
      pixel.signal_strength = src_pixel.signal_strength;
    }
    publishers_.at(protocol).publish(g_msg_tofmframe0);
  });
}

} // namespace tofsensem

#include "protocols.h"

NLT_ProtocolAnchorFrame0::NLT_ProtocolAnchorFrame0()
    : NLinkProtocol(
          true, nlt_anchorframe0_.fixed_part_size,
          {nlt_anchorframe0_.frame_header, nlt_anchorframe0_.function_mark},
          {nlt_anchorframe0_.tail_check}) {}

void NLT_ProtocolAnchorFrame0::UnpackFrameData(const uint8_t *data) {
  nlt_anchorframe0_.UnpackData(data, length());
}

bool NLT_ProtocolAnchorFrame0::Verify(const uint8_t *data) {
  return data[length() - 1] == nlt_anchorframe0_.tail_check;
}

NLT_ProtocolTagFrame0::NLT_ProtocolTagFrame0()
    : NLinkProtocol(
          true, g_nlt_tagframe0.fixed_part_size,
          {g_nlt_tagframe0.frame_header, g_nlt_tagframe0.function_mark}) {}

void NLT_ProtocolTagFrame0::UnpackFrameData(const uint8_t *data) {
  g_nlt_tagframe0.UnpackData(data, length());
}

NLT_ProtocolNodeFrame0::NLT_ProtocolNodeFrame0()
    : NLinkProtocolVLength(
          true, g_nlt_nodeframe0.fixed_part_size,
          {g_nlt_nodeframe0.frame_header, g_nlt_nodeframe0.function_mark}) {}

void NLT_ProtocolNodeFrame0::UnpackFrameData(const uint8_t *data) {
  g_nlt_nodeframe0.UnpackData(data, length());
}

NLT_ProtocolNodeFrame1::NLT_ProtocolNodeFrame1()
    : NLinkProtocolVLength(
          true, g_nlt_nodeframe1.fixed_part_size,
          {g_nlt_nodeframe1.frame_header, g_nlt_nodeframe1.function_mark}) {}

void NLT_ProtocolNodeFrame1::UnpackFrameData(const uint8_t *data) {
  g_nlt_nodeframe1.UnpackData(data, length());
}

NLT_ProtocolNodeFrame2::NLT_ProtocolNodeFrame2()
    : NLinkProtocolVLength(
          true, g_nlt_nodeframe2.fixed_part_size,
          {g_nlt_nodeframe2.frame_header, g_nlt_nodeframe2.function_mark}) {}

void NLT_ProtocolNodeFrame2::UnpackFrameData(const uint8_t *data) {
  g_nlt_nodeframe2.UnpackData(data, length());
}

NLT_ProtocolNodeFrame3::NLT_ProtocolNodeFrame3()
    : NLinkProtocolVLength(
          true, g_nlt_nodeframe3.fixed_part_size,
          {g_nlt_nodeframe3.frame_header, g_nlt_nodeframe3.function_mark}) {}

void NLT_ProtocolNodeFrame3::UnpackFrameData(const uint8_t *data) {
  g_nlt_nodeframe3.UnpackData(data, length());
}

NLT_ProtocolSettingFrame0::NLT_ProtocolSettingFrame0()
    : NLinkProtocol(
          true, g_nlt_settingframe0.fixed_part_size,
          {g_nlt_settingframe0.frame_header, g_nlt_settingframe0.function_mark}) {}

void NLT_ProtocolSettingFrame0::UnpackFrameData(const uint8_t *data) {
  g_nlt_settingframe0.UnpackData(data, length());
}

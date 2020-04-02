#include "linktrack_protocols.h"

NLT_ProtocolAnchorFrame0::NLT_ProtocolAnchorFrame0()
    : NProtocolBase(
          true, nltAnchorFrame0_.kFixedFrameLength,
          {nltAnchorFrame0_.kFrameHeader, nltAnchorFrame0_.kFunctionMark},
          {nltAnchorFrame0_.kTailCheck}) {}

void NLT_ProtocolAnchorFrame0::updateData(const uint8_t *data) {
  nltAnchorFrame0_.unpackData(data, length());
}

bool NLT_ProtocolAnchorFrame0::verify(const uint8_t *data) {
  return data[length() - 1] == nltAnchorFrame0_.kTailCheck;
}

NLT_ProtocolTagFrame0::NLT_ProtocolTagFrame0()
    : NProtocolBase(true, nltTagFrame0_.kFixedFrameLength,
                    {nltTagFrame0_.kFrameHeader, nltTagFrame0_.kFunctionMark}) {
}

void NLT_ProtocolTagFrame0::updateData(const uint8_t *data) {
  nltTagFrame0_.unpackData(data, length());
}

NLT_ProtocolNodeFrame0::NLT_ProtocolNodeFrame0()
    : NLT_ProtocolNodeFrameBase(
          true, nltNodeFrame0_.kFixedFrameLength,
          {nltNodeFrame0_.kFrameHeader, nltNodeFrame0_.kFunctionMark}) {}

void NLT_ProtocolNodeFrame0::updateData(const uint8_t *data) {
  nltNodeFrame0_.unpackData(data, length());
}

NLT_ProtocolNodeFrame1::NLT_ProtocolNodeFrame1()
    : NLT_ProtocolNodeFrameBase(
          true, nltNodeFrame1_.kFixedFrameLength,
          {nltNodeFrame1_.kFrameHeader, nltNodeFrame1_.kFunctionMark}) {}

void NLT_ProtocolNodeFrame1::updateData(const uint8_t *data) {
  nltNodeFrame1_.unpackData(data, length());
}

NLT_ProtocolNodeFrame2::NLT_ProtocolNodeFrame2()
    : NLT_ProtocolNodeFrameBase(
          true, nltNodeFrame2_.kFixedFrameLength,
          {nltNodeFrame2_.kFrameHeader, nltNodeFrame2_.kFunctionMark}) {}

void NLT_ProtocolNodeFrame2::updateData(const uint8_t *data) {
  nltNodeFrame2_.unpackData(data, length());
}

NLT_ProtocolNodeFrame3::NLT_ProtocolNodeFrame3()
    : NLT_ProtocolNodeFrameBase(
          true, nltNodeFrame3_.kFixedFrameLength,
          {nltNodeFrame3_.kFrameHeader, nltNodeFrame3_.kFunctionMark}) {}

void NLT_ProtocolNodeFrame3::updateData(const uint8_t *data) {
  nltNodeFrame3_.unpackData(data, length());
}

bool NLT_ProtocolNodeFrameBase::updateLength(const uint8_t *data,
                                             size_t availableBytes) {
  if (availableBytes < 4)
    return false;
  return setLength(static_cast<size_t>(data[2] | data[3] << 8));
}

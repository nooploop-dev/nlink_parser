#include "tofsense_protocols.h"

NTS_ProtocolFrame0::NTS_ProtocolFrame0()
    : NProtocolBase(true, ntsFrame0_.kFixedFrameLength,
                    {ntsFrame0_.kFrameHeader, ntsFrame0_.kFunctionMark}) {}

void NTS_ProtocolFrame0::updateData(const uint8_t *data) {
  ntsFrame0_.unpackData(data, length());
}

#ifndef NLINK_TOFSENSE_PROTOCOLS_H
#define NLINK_TOFSENSE_PROTOCOLS_H

#include "nlink_unpack/nlink_tofsense_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nprotocolbase.h"

class NTS_ProtocolFrame0 : public NProtocolBase {
public:
  NTS_ProtocolFrame0();

protected:
  void updateData(const uint8_t *data) override;
};

#endif // NLINK_TOFSENSE_PROTOCOLS_H

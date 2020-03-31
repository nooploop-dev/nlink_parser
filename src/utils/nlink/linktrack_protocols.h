#ifndef NLINK_LINKTRACK_PROTOCOLS_H
#define NLINK_LINKTRACK_PROTOCOLS_H

#include "nlink_unpack/nlink_linktrack_anchorframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe1.h"
#include "nlink_unpack/nlink_linktrack_nodeframe2.h"
#include "nlink_unpack/nlink_linktrack_nodeframe3.h"
#include "nlink_unpack/nlink_linktrack_tagframe0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nprotocolbase.h"

class NLT_ProtocolAnchorFrame0 : public NProtocolBase {
public:
  NLT_ProtocolAnchorFrame0();

protected:
  void updateData(const uint8_t *data) override;
  bool verify(const uint8_t *data) override;
};

class NLT_ProtocolTagFrame0 : public NProtocolBase {
public:
  NLT_ProtocolTagFrame0();

protected:
  void updateData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrameBase : public NProtocolBase {
public:
  using NProtocolBase::NProtocolBase;
  ~NLT_ProtocolNodeFrameBase() override = default;

protected:
  bool updateLength(const uint8_t *data, size_t availableBytes) override;
};

class NLT_ProtocolNodeFrame0 : public NLT_ProtocolNodeFrameBase {
public:
  NLT_ProtocolNodeFrame0();

protected:
  void updateData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame1 : public NLT_ProtocolNodeFrameBase {
public:
  NLT_ProtocolNodeFrame1();

protected:
  void updateData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame2 : public NLT_ProtocolNodeFrameBase {
public:
  NLT_ProtocolNodeFrame2();

protected:
  void updateData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame3 : public NLT_ProtocolNodeFrameBase {
public:
  NLT_ProtocolNodeFrame3();

protected:
  void updateData(const uint8_t *data) override;
};

#endif // NLINK_LINKTRACK_PROTOCOLS_H

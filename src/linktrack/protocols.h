#ifndef LINKTRACK_PROTOCOLS_H
#define LINKTRACK_PROTOCOLS_H

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_linktrack_anchorframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe1.h"
#include "nlink_unpack/nlink_linktrack_nodeframe2.h"
#include "nlink_unpack/nlink_linktrack_nodeframe3.h"
#include "nlink_unpack/nlink_linktrack_nodeframe4.h"
#include "nlink_unpack/nlink_linktrack_nodeframe5.h"
#include "nlink_unpack/nlink_linktrack_nodeframe6.h"
#include "nlink_unpack/nlink_linktrack_tagframe0.h"

class NLT_ProtocolAnchorFrame0 : public NLinkProtocol
{
public:
  NLT_ProtocolAnchorFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
  bool Verify(const uint8_t *data) override;
};

class NLT_ProtocolTagFrame0 : public NLinkProtocol
{
public:
  NLT_ProtocolTagFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame0 : public NLinkProtocolVLength
{
public:
  NLT_ProtocolNodeFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame1 : public NLinkProtocolVLength
{
public:
  NLT_ProtocolNodeFrame1();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame2 : public NLinkProtocolVLength
{
public:
  NLT_ProtocolNodeFrame2();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame3 : public NLinkProtocolVLength
{
public:
  NLT_ProtocolNodeFrame3();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame4 : public NLinkProtocolVLength
{
public:
  NLT_ProtocolNodeFrame4();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame5 : public NLinkProtocolVLength
{
public:
  NLT_ProtocolNodeFrame5();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

class NLT_ProtocolNodeFrame6 : public NLinkProtocolVLength
{
public:
  NLT_ProtocolNodeFrame6();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

#endif // LINKTRACK_PROTOCOLS_H

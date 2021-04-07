#ifndef NLINK_PROTOCOL_H
#define NLINK_PROTOCOL_H
#include <functional>

#include "protocol_extracter/nprotocol_base.h"

class NLinkProtocol : public NProtocolBase
{
public:
  using NProtocolBase::NProtocolBase;

  void SetHandleDataCallback(std::function<void()> handle)
  {
    HandleDataCallback_ = handle;
  }

protected:
  void HandleData(const uint8_t *data) final;
  virtual void UnpackFrameData(const uint8_t *data) = 0;
  bool Verify(const uint8_t *data) override;

private:
  std::function<void()> HandleDataCallback_;
};

class NLinkProtocolVLength : public NLinkProtocol
{
public:
  using NLinkProtocol::NLinkProtocol;

protected:
  bool UpdateLength(const uint8_t *data, size_t available_bytes) override;
};

#endif // NLINK_PROTOCOL_H

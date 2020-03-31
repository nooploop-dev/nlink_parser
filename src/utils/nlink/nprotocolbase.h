#ifndef NPROTOCOLBATH_H
#define NPROTOCOLBATH_H

#include <functional>
#include <stddef.h>
#include <stdint.h>
#include <string>

class NProtocolBase {
public:
  NProtocolBase(bool isLengthKnowable, size_t length,
                const std::initializer_list<uint8_t> &header,
                const std::initializer_list<uint8_t> &tail = {});
  virtual ~NProtocolBase() = default;

  void setDataUseHandle(std::function<void()> handle) { useData_ = handle; }

protected:
  virtual bool updateLength(const uint8_t *data, size_t availableBytes);

  virtual bool verify(const uint8_t *data);

  virtual void updateData(const uint8_t *data) = 0;

  size_t length() const { return length_; }

  void setLength(size_t length) { length_ = length; }

  bool isLengthKnowable() const { return isLengthKnowable_; }

  std::string fixedHeader() const { return header_; }

  std::string fixedTail() const { return tail_; }

private:
  //例如nmea协议长度不可知，直到找到tail
  bool isLengthKnowable_ = true;
  size_t length_ = 0;
  std::string header_;
  std::string tail_;

  std::function<void()> useData_;

  friend class NFrameExtraction;
};

#endif // NPROTOCOLBATH_H

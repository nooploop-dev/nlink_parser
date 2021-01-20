#ifndef NPROTOCOL_BASE_H
#define NPROTOCOL_BASE_H

#include <stddef.h>
#include <stdint.h>

#include <string>

class NProtocolBase {
 public:
  NProtocolBase(bool is_length_knowable, size_t fixed_part_size,
                const std::initializer_list<uint8_t> &header,
                const std::initializer_list<uint8_t> &tail = {});
  virtual ~NProtocolBase() = default;

  size_t length() const { return length_; }

  bool is_length_knowable() const { return is_length_knowable_; }

  std::string fixed_header() const { return fixed_header_; }

  std::string fixed_tail() const { return fixed_tail_; }

 protected:
  virtual bool UpdateLength(const uint8_t *data, size_t availableBytes);

  virtual bool Verify(const uint8_t *data);

  virtual void HandleData(const uint8_t *data) = 0;

  bool set_length(size_t length) {
    if (length < fixed_part_size_) {
      return false;
    }
    length_ = length;
    return true;
  }

 private:
  bool is_length_knowable_;
  const size_t fixed_part_size_;
  size_t length_;
  std::string fixed_header_;
  std::string fixed_tail_;

  friend class NProtocolExtracter;
};

#endif  // NPROTOCOL_BASE_H

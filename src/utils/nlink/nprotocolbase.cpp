#include "nprotocolbase.h"
#include <numeric>

NProtocolBase::NProtocolBase(bool isLengthKnowable, size_t length,
                             const std::initializer_list<uint8_t> &header,
                             const std::initializer_list<uint8_t> &tail)
    : isLengthKnowable_(isLengthKnowable), length_(length) {
  for (auto byte : header) {
    header_.push_back(static_cast<char>(byte));
  }
  for (auto byte : tail) {
    tail_.push_back(static_cast<char>(byte));
  }
}

bool NProtocolBase::updateLength(const uint8_t *, size_t) { return true; }

bool NProtocolBase::verify(const uint8_t *data) {
  uint8_t sum = 0;
  return data[length() - 1] ==
         std::accumulate(data, data + length() - sizeof(sum), sum);
}

#include "nprotocol_base.h"

#include <numeric>

NProtocolBase::NProtocolBase(bool is_length_knowable, size_t fixed_part_size,
                             const std::initializer_list<uint8_t> &header,
                             const std::initializer_list<uint8_t> &tail)
    : is_length_knowable_(is_length_knowable),
      fixed_part_size_(fixed_part_size),
      length_(fixed_part_size) {
  for (auto byte : header) {
    fixed_header_.push_back(static_cast<char>(byte));
  }

  for (auto byte : tail) {
    fixed_tail_.push_back(static_cast<char>(byte));
  }
}

bool NProtocolBase::UpdateLength(const uint8_t *, size_t) { return true; }

bool NProtocolBase::Verify(const uint8_t *data) { return true; }

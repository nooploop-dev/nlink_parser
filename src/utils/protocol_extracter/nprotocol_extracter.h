#ifndef NPROTOCOL_EXTRACTER_H
#define NPROTOCOL_EXTRACTER_H

#include <list>
#include <vector>

#include "nprotocol_base.h"

class NProtocolExtracter {
 public:
  void AddProtocol(NProtocolBase *protocol);

  void RemoveProtocol(NProtocolBase *protocol);

  void AddNewData(const uint8_t *data, size_t data_length);

  void AddNewData(const std::string &data);

 private:
  std::vector<NProtocolBase *> protocols_;

  std::string prev_data_array_;

  int max_header_size_ = 0;
};

#endif  // NPROTOCOL_EXTRACTER_H

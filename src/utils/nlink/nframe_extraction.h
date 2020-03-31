#ifndef NFRAMEEXTRACTION_H
#define NFRAMEEXTRACTION_H

#include "nprotocolbase.h"
#include <list>
#include <vector>

class NFrameExtraction {
public:
  NFrameExtraction();
  ~NFrameExtraction();

  void addProtocol(NProtocolBase *protocol);

  void unpackData(const uint8_t *newData, size_t dataLength);
  void unpackData(const std::string &newData);

private:
  std::vector<NProtocolBase *> protocols_;

  std::string prevDataArray_;

  typedef struct {
    NProtocolBase *protocol;
    size_t headerIndex;
  } SortInfo;

  std::vector<SortInfo> sortInfos_;

  bool isAnalysingData_ = false;
};

#endif // NFRAMEEXTRACTION_H

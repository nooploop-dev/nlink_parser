#include "nframe_extraction.h"
#include <algorithm>
#include <cassert>

NFrameExtraction::NFrameExtraction() {}

NFrameExtraction::~NFrameExtraction() {
  for (auto protocol : protocols_) {
    delete protocol;
  }
}

void NFrameExtraction::addProtocol(NProtocolBase *protocol) {
  assert(std::find(protocols_.begin(), protocols_.end(), protocol) ==
         protocols_.end());
  protocols_.push_back(protocol);
}

void NFrameExtraction::unpackData(const uint8_t *newData, size_t dataLength) {
  unpackData(std::string(reinterpret_cast<const char *>(newData), dataLength));
}

void NFrameExtraction::unpackData(const std::string &newData) {
  if (newData.empty())
    return;

  auto dataArray = prevDataArray_ + newData;
  prevDataArray_.clear();

  //记录首次出现的因为数据不足导致无法验证的协议帧头位置
  auto incompleteIndex_ = dataArray.size();
  sortInfos_.clear();
  for (size_t i = 0; i < protocols_.size(); ++i) {
    auto protocol = protocols_.at(i);

    const auto &headerArray = protocol->fixedHeader();

    auto headerIndex = -headerArray.size();
    for (;;) {
      headerIndex =
          dataArray.find(headerArray, headerIndex + headerArray.size());
      if (headerIndex != std::string::npos) {
        sortInfos_.push_back(SortInfo{protocol, headerIndex});
      } else
        break;
    }
  }
  //根据帧头所在位置对协议进行排序
  std::sort(sortInfos_.begin(), sortInfos_.end(), [](SortInfo a, SortInfo b) {
    if (a.headerIndex < b.headerIndex)
      return true;
    else if (a.headerIndex == b.headerIndex) {
      if (a.protocol->length() < b.protocol->length()) {
        return true;
      }
    }
    return false;
  });

  size_t indexBegin = 0;
  for (auto iter = sortInfos_.begin(); iter != sortInfos_.end(); ++iter) {
    auto &sortInfo = *iter;
    auto headerIndex = sortInfo.headerIndex;
    if (headerIndex < indexBegin)
      return;
    auto protocol = sortInfo.protocol;
    auto dataBegin =
        reinterpret_cast<const uint8_t *>(dataArray.data()) + headerIndex;
    auto availableBytes = dataArray.size() - headerIndex;
    if (protocol->isLengthKnowable()) {

      //更新协议长度
      if (!protocol->updateLength(dataBegin, availableBytes)) {
        if (incompleteIndex_ == dataArray.size()) {
          incompleteIndex_ = headerIndex;
        }
        continue;
      }
      //从当前协议帧头在temparray中位置开始算起，数据不够协议长度时直接进行下一帧协议计算
      if (availableBytes < protocol->length()) {
        if (incompleteIndex_ == dataArray.size()) {
          incompleteIndex_ = headerIndex;
        }
        continue;
      }

    } else {
      //两头封闭协议如NMEA无法得知协议长度，只能通过两头校验项初步截取协议数据
      auto headerByteArray = protocol->fixedHeader();
      auto tailByteArray = protocol->fixedTail();
      auto tailIndex =
          dataArray.find(tailByteArray, headerIndex + headerByteArray.size());
      if (tailIndex == std::string::npos) {
        if (incompleteIndex_ == dataArray.size()) {
          incompleteIndex_ = headerIndex;
        }
        continue;
      }
      protocol->setLength(tailIndex - headerIndex + tailByteArray.size());
    }

    //协议校验
    if (!protocol->verify(dataBegin)) {
      continue;
    }
    //更新协议数据
    protocol->updateData(dataBegin);
    //使用协议数据
    if (protocol->useData_) {
      protocol->useData_();
    }

    //有协议成功,覆盖前面因为协议字节数不足而做的标记
    incompleteIndex_ = dataArray.size();
    //跳过当前校验成功协议的长度范围内检测到帧头存在的协议
    indexBegin = headerIndex + protocol->length();
  }
  //保存当前因协议数据不足而无法判断的数据
  if (incompleteIndex_ < dataArray.size()) {
    prevDataArray_ = dataArray.substr(incompleteIndex_);
  } else {
    //比如最后剩一个55，不会被检测到，但是这个必须保留到下一次，否则可能会丢帧，现在是无论什么情况都保留了，待优化
    prevDataArray_.clear();
    prevDataArray_.push_back(dataArray.back());
  }
}

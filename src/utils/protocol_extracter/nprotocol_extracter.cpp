#include "nprotocol_extracter.h"

#include <algorithm>
#include <cassert>
#include <iostream>

/**
 * @brief 将协议添加到 NProtocolExtracter 中，使该协议获取解析支持。
 *
 * @param protocol
 */
void NProtocolExtracter::AddProtocol(NProtocolBase *protocol) {
  assert(std::find(protocols_.begin(), protocols_.end(), protocol) ==
         protocols_.end());
  protocols_.push_back(protocol);
  if (protocol->fixed_header().size() > max_header_size_) {
    max_header_size_ = protocol->fixed_header().size();
  }
}

/**
 * @brief 从 NProtocolExtracter 中移除协议，将不再检测该协议数据。
 *
 * @param protocol
 */
void NProtocolExtracter::RemoveProtocol(NProtocolBase *protocol) {
  auto index = std::find(protocols_.begin(), protocols_.end(), protocol);
  assert(index != protocols_.end());
  protocols_.erase(index);

  int max_size = 0;
  for (auto p : protocols_) {
    if (p->fixed_header().size() > max_size) {
      max_size = p->fixed_header().size();
    }
  }
  max_header_size_ = max_size;
}

/**
 * @brief 添加新获取的数据到 NProtocolExtracter
 * 中进行处理，如数据帧不够长将自动进行拼接。
 *
 * @param data
 * @param data_length
 */
void NProtocolExtracter::AddNewData(const uint8_t *data, size_t data_length) {
  AddNewData(std::string(reinterpret_cast<const char *>(data), data_length));
}

struct SortInfo {
  NProtocolBase *protocol;
  size_t header_index;
};

void printfString(std::string s)
{
  std::cout << "========================="<<std::endl;

  std::string ms  = "";
  const char *p  = s.c_str();
  for(int i=0;i<s.length();i++)
  {
     char buff[8] = {0};
     sprintf(buff,"%02x", p[i]&0xff);
     ms += buff;
  }
  std::cout << ms.c_str() << std::endl;

  std::cout<<"-----------------------"<<std::endl;
}

void NProtocolExtracter::AddNewData(const std::string &data) {
  if (data.empty()) return;

  auto data_array = prev_data_array_ + data;
  prev_data_array_.clear();
  // printfString(data_array); // for test

  //记录因为数据不足导致无法进行校验的帧头位置，需要拼接到下一次数据进行提取
  int incomplete_index = data_array.size();

  //提取帧头
  std::vector<SortInfo> sort_infos;
  for (size_t i = 0; i < protocols_.size(); ++i) {
    auto protocol = protocols_.at(i);

    const auto &fixed_header = protocol->fixed_header();

    auto header_index = -fixed_header.size();
    for (;;) {
      header_index =
          data_array.find(fixed_header, header_index + fixed_header.size());
      if (header_index != std::string::npos) {
        sort_infos.push_back(SortInfo{protocol, header_index});
      } else
        break;
    }
  }

  //检查协议头并排列顺序
  std::sort(sort_infos.begin(), sort_infos.end(), [](SortInfo a, SortInfo b) {
    if (a.header_index < b.header_index)
      return true;
    else if (a.header_index == b.header_index) {
      if (a.protocol->length() < b.protocol->length()) {
        return true;
      }
    }
    return false;
  });

  //表示需要处理的数据位置
  int index_begin = 0;

  for (const auto &sort_info : sort_infos) {
    auto header_index = sort_info.header_index;
    if (header_index < index_begin) continue;
    auto protocol = sort_info.protocol;
    auto data_begin =
        reinterpret_cast<const uint8_t *>(data_array.data()) + header_index;
    auto available_bytes = data_array.size() - header_index;
    if (protocol->is_length_knowable()) {
      //绝大部分协议都长度可知，变长协议也可通过协议中表示长度的部分计算获取
      if (!protocol->UpdateLength(data_begin, available_bytes)) {
        if (incomplete_index == data_array.size()) {
          incomplete_index = header_index;
        }
        continue;
      }
      if (available_bytes < protocol->length()) {
        if (incomplete_index == data_array.size()) {
          incomplete_index = header_index;
        }
        continue;
      }
    } else {
      //对于nmea类型协议，仅依靠固定的帧头及帧尾进行判断，直到找到帧尾才可知当前帧实际长度
      auto fixed_header = protocol->fixed_header();
      auto fixed_tail = protocol->fixed_tail();
      auto tail_index =
          data_array.find(fixed_tail, header_index + fixed_header.size());
      if (tail_index == std::string::npos) {
        if (incomplete_index == data_array.size()) {
          incomplete_index = header_index;
        }
        continue;
      }
      protocol->set_length(tail_index - header_index + fixed_tail.size());
    }

    //协议校验
    if (!protocol->Verify(data_begin)) {
      continue;
    }
    //处理协议数据
    protocol->HandleData(data_begin);

    incomplete_index = data_array.size();

    index_begin = header_index + protocol->length();
  }

  // std::cout << "data_array.size(): " << data_array.size() << "\n";

  //处理末尾数据，视情况需要保留到下一次计算
  if (incomplete_index + max_header_size_ - 1 <= data_array.size()) {
    prev_data_array_ = data_array.substr(incomplete_index);
  } else {
    if (index_begin + max_header_size_ - 1 <= data_array.size()) {
      prev_data_array_ =
          data_array.substr(data_array.size() + 1 - max_header_size_);
    } else {
      prev_data_array_ = data_array.substr(index_begin);
    }
  }

  // std::cout << "prev_data_array_.size(): " << prev_data_array_.size() <<
  // "\n";
}

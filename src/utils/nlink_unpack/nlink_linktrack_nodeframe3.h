#ifndef NLINK_LINKTRACK_NODEFRAME3_H
#define NLINK_LINKTRACK_NODEFRAME3_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "nlink_typedef.h"

  typedef struct
  {
    linktrack_role_e role;
    uint8_t id;
    float dis;
    float fp_rssi;
    float rx_rssi;
  } nlt_nodeframe3_node_t;

  typedef struct
  {
    linktrack_role_e role;
    uint8_t id;
    uint32_t local_time;
    uint32_t system_time;
    float voltage;
    uint8_t valid_node_count;
    nlt_nodeframe3_node_t *nodes[256];
  } nlt_nodeframe3_result_t;

  typedef struct
  {
    const size_t fixed_part_size;
    const uint8_t frame_header;
    const uint8_t function_mark;
    nlt_nodeframe3_result_t result;

    uint8_t (*const UnpackData)(const uint8_t *data, size_t data_length);
  } nlt_nodeframe3_t;

  extern nlt_nodeframe3_t g_nlt_nodeframe3;

#ifdef __cplusplus
}
#endif
#endif // NLINK_LINKTRACK_NODEFRAME3_H

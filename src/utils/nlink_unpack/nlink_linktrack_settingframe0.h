#ifndef NLINK_LINKTRACK_SETTINGFRAME0_H
#define NLINK_LINKTRACK_SETTINGFRAME0_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "nlink_typedef.h"

  typedef struct
  {
    float ag_3d[3];
  } nlt_settingframe0_node_t;

  typedef struct
  {
    uint8_t mix;
    linktrack_role_e role;
    uint8_t id;
    uint8_t group;
    uint8_t valid_node_count;
    nlt_settingframe0_node_t *nodes[10];
  } nlt_settingframe0_result_t;

  typedef struct
  {
    const size_t fixed_part_size;
    const uint8_t frame_header;
    const uint8_t function_mark;
    nlt_settingframe0_result_t result;
    const uint8_t sum_check;
    
    uint8_t (*const UnpackData)(const uint8_t *data, size_t data_length);
  } nlt_settingframe0_t;

  extern nlt_settingframe0_t g_nlt_settingframe0;

#ifdef __cplusplus
}
#endif

#endif // NLINK_LINKTRACK_SETTINGFRAME0_H

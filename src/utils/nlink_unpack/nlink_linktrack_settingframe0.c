#include "nlink_linktrack_settingframe0.h"

#include "nlink_utils.h"
#include <stdio.h>
#include <string.h>

NLINK_PACKED(typedef struct {
  nint24_t ag_3d[3];
})
nlt_settingframe0_node_raw_t;

NLINK_PACKED(typedef struct {
  uint8_t header[2];
  uint8_t mix;
  uint8_t role;
  uint8_t reserved1[5];
  uint8_t id;
  uint8_t reserved2[26];
  uint8_t group;
  nlt_settingframe0_node_raw_t nodes[10];
  uint8_t sumCheck;
})
nlt_settingframe0_raw_t;

static nlt_settingframe0_raw_t g_frame;

static uint8_t UnpackData(const uint8_t *data, size_t data_length)
{
  if (data_length < g_nlt_settingframe0.fixed_part_size ||
      data[0] != g_nlt_settingframe0.frame_header ||
      data[1] != g_nlt_settingframe0.function_mark)
    return 0;
  // size_t frame_length = NLINK_PROTOCOL_LENGTH(data);
  size_t frame_length = g_nlt_settingframe0.fixed_part_size;
  if (data_length < frame_length)
    return 0;
  if (!NLINK_VerifyCheckSum(data, frame_length))
    return 0;
  
  static uint8_t init_needed = 1;
  if (init_needed)
  {
    memset(g_nlt_settingframe0.result.nodes, 0,
           sizeof(g_nlt_settingframe0.result.nodes));
    init_needed = 0;
  }

  memcpy(&g_frame, data, g_nlt_settingframe0.fixed_part_size);
  g_nlt_settingframe0.result.mix = g_frame.mix;
  g_nlt_settingframe0.result.role = g_frame.role;
  g_nlt_settingframe0.result.id = g_frame.id;
  g_nlt_settingframe0.result.group = g_frame.group;
  
  g_nlt_settingframe0.result.valid_node_count = 0;
  for (size_t i = 0, count = ARRAY_LENGTH(g_nlt_settingframe0.result.nodes); i < count; ++i)
  {
    uint8_t index = g_nlt_settingframe0.result.valid_node_count;
    TRY_MALLOC_NEW_NODE(g_nlt_settingframe0.result.nodes[index], nlt_settingframe0_node_t)

    NLINK_TRANSFORM_ARRAY_INT24(g_nlt_settingframe0.result.nodes[index]->ag_3d, g_frame.nodes[i].ag_3d, MULTIPLY_POS)
    ++g_nlt_settingframe0.result.valid_node_count;
  }
  return 1;
}

nlt_settingframe0_t g_nlt_settingframe0 = {.fixed_part_size = 128,
                                           .frame_header = 0x54,
                                           .function_mark = 0x00,
                                           .UnpackData = UnpackData};

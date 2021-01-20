#include "nlink_linktrack_nodeframe0.h"
#include "nlink_linktrack_nodeframe1.h"
#include "nlink_tofsense_frame0.h"
#include "nlink_utils.h"

NLINK_PACKED(typedef struct
             {
               uint8_t a;
               uint8_t b;
               uint32_t c;
               double d;
               uint8_t e;
             })
pack_test_t;

int main()
{
  {
    uint32_t check = 1;
    if (*(uint8_t *)(&check) != 1)
    {
      printf("Error: this code must run in little endian.");
      return EXIT_FAILURE;
    }
  }

  if (sizeof(pack_test_t) != 15)
  {
    printf("Error: Pack do not work, pack size:%d. Contact us for support", sizeof(pack_test_t));
    return EXIT_FAILURE;
  }

  uint8_t data[1024];
  size_t data_length;
  {
    const char *string = "57 00 ff 00 c2 45 00 00 80 02 00 00 08 00 ff e6";
    data_length = NLink_StringToHex(string, data);
    if (g_nts_frame0.UnpackData(data, data_length))
    {
      printf("TOFSense Frame0 data unpack successfully:\r\n");
      printf("id:%d, distance:%f\r\n", g_nts_frame0.result.id,
             g_nts_frame0.result.dis);
    }
  }
  {
    const char *string =
        "55 02 42 00 01 00 d1 2c c3 88 02 02 00 09 00 11 22 33 44 55 66 77 "
        "88 99 02 02 25 00 11 12 23 22 32 44 34 54 55 65 67 76 67 87 77 99 "
        "aa a2 13 45 57 65 56 56 56 56 57 78 43 33 34 44 44 44 44 46 76 0d";

    data_length = NLink_StringToHex(string, data);
    if (g_nlt_nodeframe0.UnpackData(data, data_length))
    {
      nlt_nodeframe0_result_t *result = &g_nlt_nodeframe0.result;
      printf("LinkTrack NodeFrame0 data unpack successfully:\r\n");
      printf("valid_node_count:%d\r\n", result->valid_node_count);
      for (int i = 0; i < result->valid_node_count; ++i)
      {
        nlt_nodeframe0_node_t *node = result->nodes[i];
        printf("role:%d, id:%d, data_length:%d\r\n", node->role, node->id,
               node->data_length);
      }
    }
  }
  {
    const char *string =
        "55 03 44 00 03 00 e8 80 00 00 00 86 00 00 01 51 01 e8 01 f2 02 02 92 "
        "09 48 13 02 02 00 5f 0b 00 86 09 00 9b ff ff c9 37 8a 34 06 ee "
        "37 3f aa 02 02 93 09 00 45 09 00 c4 fc ff 8d 09 00 66 09 00 c4 fc "
        "ff 8e";
    data_length = NLink_StringToHex(string, data);
    if (g_nlt_nodeframe1.UnpackData(data, data_length))
    {
      nlt_nodeframe1_result_t *result = &g_nlt_nodeframe1.result;
      printf("LinkTrack NodeFrame1 data unpack successfully:\r\n");
      printf("id:%d, system_time:%d, valid_node_count:%d\r\n", result->id,
             result->system_time, result->valid_node_count);
      for (int i = 0; i < result->valid_node_count; ++i)
      {
        nlt_nodeframe1_node_t *node = result->nodes[i];
        printf("role:%d, id:%d, x:%f, y:%f\r\n", node->role, node->id,
               node->pos_3d[0], node->pos_3d[1]);
      }
    }
  }

  return EXIT_SUCCESS;
}

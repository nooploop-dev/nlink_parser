// gtest
#include "../src/iot/init.h"
#include "../src/linktrack/init.h"
#include "../src/linktrack_aoa/init.h"
#include "../src/tofsense/init.h"
#include "../src/tofsensem/init.h"
#include <gtest/gtest.h>
#include <nlink_parser/IotFrame0.h>
#include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe1.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackNodeframe4.h>
#include <nlink_parser/LinktrackNodeframe5.h>
#include <nlink_parser/LinktrackNodeframe6.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include <nlink_parser/TofsenseFrame0.h>
#include <nlink_parser/TofsenseMFrame0.h>
#include <vector>

static const double kAbsError = 0.001;

#define COMPARE_ARRAY(SRC, DEST, kAbsError)                                    \
  {                                                                            \
    const auto &ACTUAL = SRC;                                                  \
    const auto &EXPECTED = std::vector<float> DEST;                            \
    for (size_t _INDEX = 0; _INDEX < EXPECTED.size(); ++_INDEX)                \
    {                                                                          \
      EXPECT_NEAR(EXPECTED.at(_INDEX), ACTUAL.at(_INDEX), kAbsError);          \
    }                                                                          \
  }

namespace linktrack
{
  extern nlink_parser::LinktrackAnchorframe0 g_msg_anchorframe0;
  extern nlink_parser::LinktrackTagframe0 g_msg_tagframe0;
  extern nlink_parser::LinktrackNodeframe0 g_msg_nodeframe0;
  extern nlink_parser::LinktrackNodeframe1 g_msg_nodeframe1;
  extern nlink_parser::LinktrackNodeframe2 g_msg_nodeframe2;
  extern nlink_parser::LinktrackNodeframe3 g_msg_nodeframe3;
  extern nlink_parser::LinktrackNodeframe4 g_msg_nodeframe4;
  extern nlink_parser::LinktrackNodeframe5 g_msg_nodeframe5;
  extern nlink_parser::LinktrackNodeframe6 g_msg_nodeframe6;
} // namespace linktrack

TEST(NLinkParser, linktrack)
{
  NProtocolExtracter protocol_extraction;
  linktrack::Init init(&protocol_extraction, nullptr);
  uint8_t data[1024];
  {
    auto string = "55 00 00 02 4f 0b 00 73 09 00 f9 fe ff 6c 01 4e 01 ea 01 ed "
                  "01 00 00 00 "
                  "00 00 00 00 00 ff 0b 00 8e 09 00 4a fe ff c9 37 8a 34 06 ee "
                  "37 3f aa 02 "
                  "02 7e 09 00 67 09 00 a3 02 02 83 09 00 5f 09 00 a3 fb ff 3e "
                  "01 2a 01 12 "
                  "02 13 02 00 00 00 00 00 00 00 00 ff ab 91 ef ea 45 09 e1 18 "
                  "5a 33 b4 f2 "
                  "08 40 da ca c8 fd df f7 44 d7 3c 3a ff a2 ff f6 12 0c 05 4c "
                  "2b 7b 7d 6b "
                  "49 41 1d 48 44 fd bc 96 21 43 16 45 97 ef c6 71 e5 ff f4 83 "
                  "77 fa 32 d3 "
                  "01 19 29 bd fd fb 60 ff 00 04 1a 12 38 ba 26 b7 9b 6d 7d 3a "
                  "ff 2f 5f da "
                  "82 64 40 42 86 fe df 72 08 1a 28 0b fb f7 77 f2 72 76 c4 38 "
                  "79 2a 9d ff "
                  "24 24 20 25 4e b7 9f 99 4d 70 56 80 d3 9e e2 7d 12 3a 85 c6 "
                  "8f bd c6 9f "
                  "81 8f ff f3 0c f4 c5 ce c9 91 51 d1 65 5f 30 3f 05 91 0c 86 "
                  "b0 07 7e 39 "
                  "d1 68 76 46 d3 ff 9f 6b 11 08 ce 06 b4 ac 59 e1 c4 33 ca 80 "
                  "1f be 13 bc "
                  "6d e2 32 8d ea 75 a5 b1 ff 06 95 4d f7 12 d6 61 45 8c 89 84 "
                  "36 64 96 e0 "
                  "f0 02 a2 84 6c 37 fb fa ac 18 a4 ff 79 6a 78 76 78 60 ba 00 "
                  "fb 7b d6 fb "
                  "f8 00 8a f9 b5 63 cf eb d0 45 56 43 23 e9 ff bb 73 25 0a 0b "
                  "cf b9 df cd "
                  "aa 62 1f 54 eb 1d f7 b8 c0 02 d8 8a c3 af 34 97 03 ff e2 88 "
                  "b6 c3 39 fa "
                  "89 5d 40 00 62 8b ce 4b c0 20 28 11 fd d6 ce bd 3c 60 7b 72 "
                  "ff 42 e4 ed "
                  "90 61 70 40 28 df c1 be 11 00 15 55 cb 95 56 2d 50 a3 5d 28 "
                  "32 ee bb ff "
                  "01 91 1d 67 35 f5 f8 bf 3c 10 01 1c 6d 1b f5 5e 89 1f 28 23 "
                  "60 31 2f 75 "
                  "44 02 ff 51 7b c7 af af f2 d0 0e ca bb 61 31 f6 ac e0 69 10 "
                  "34 b3 73 7f "
                  "30 14 18 01 7a ff 76 ed 42 37 66 0a 2b 55 7f f5 04 43 f4 40 "
                  "4e 3f 52 ee "
                  "c4 4f 09 1e b9 8d f6 3d ff 51 f2 20 d6 4b d2 7d 87 0c a8 15 "
                  "99 4b ee 2a "
                  "42 41 69 68 75 51 f3 bf 5d 4b 01 ff 8e 9e 1e 92 a8 a1 57 04 "
                  "f4 c4 3f eb "
                  "43 18 ac 03 58 0d 5d d4 51 c8 81 64 7e 6d ff 71 c1 18 2b 4f "
                  "b7 ef ec 7c "
                  "46 2a a4 8f 6c 94 cb 63 9e 6c 22 74 de 1f fc a3 62 ff 1f 01 "
                  "f9 61 71 50 "
                  "57 1a 83 b2 64 1e 3e 1c 85 22 2b 99 56 6d f9 bb b2 c1 a1 22 "
                  "ff ef 5c af "
                  "f7 4c f0 68 6d 4c 42 37 24 03 45 28 fd 1b a6 e2 04 7c a4 7c "
                  "e5 33 9b ff "
                  "6b 1a 03 14 77 5f 23 3d a8 6c 44 80 17 64 f1 a2 10 06 38 24 "
                  "06 d3 3c 3c "
                  "0c 68 ff 98 d2 ab 63 ca d0 f2 c6 2a 7e a3 57 dd 21 35 b1 60 "
                  "14 ee de 36 "
                  "15 d8 2b 08 b7 ff 17 6c 6b 21 ad da dc b3 31 dc ea 0c a6 cb "
                  "71 ff 4c e9 "
                  "8e 08 66 58 e2 3e 6d e6 ff 00 d8 00 1c db db d1 05 ac ee 06 "
                  "85 f6 de da "
                  "00 51 0c 2e ca ed 8b c7 16 7b 06 ff 17 ef f9 ef 7c 1e e0 d3 "
                  "3f 6d 6c 6f "
                  "eb 27 b9 65 53 c6 1f 2a 3e 11 3a 40 8f b1 ff 3e a2 12 20 c3 "
                  "d5 b7 68 be "
                  "18 14 5c a4 a1 7f bb 9f fa c4 54 80 38 65 4e ff 51 ff 9a 1b "
                  "d7 3f e7 96 "
                  "20 3b 1c 08 82 9b 3f e2 04 ae 16 80 b7 ca e3 07 83 60 50 f5 "
                  "ff 8a fd c1 "
                  "24 2d 84 35 b7 fe 0c 6d 9b d6 e8 8c ee 79 21 ef e1 10 82 90 "
                  "b7 92 c2 ff "
                  "57 04 da 00 0e bf 4e e7 c2 70 aa cc 95 85 ff 6b 30 03 83 41 "
                  "8b df ad e3 "
                  "8a 40 ff 02 ed cb 6f 7e 13 20 99 f1 f1 87 2a 21 82 00 00 0c "
                  "c7 e9 7a 83 "
                  "13 00 7d 00 00 00 03 ee";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);

    auto &msgData = linktrack::g_msg_anchorframe0;
    COMPARE_ARRAY(msgData.nodes[0].pos_3d, ({2.895f, 2.419f, -0.263f}),
                  kAbsError)
    COMPARE_ARRAY(msgData.nodes[0].dis_arr,
                  ({3.64f, 3.34f, 4.9f, 4.93f, 0, 0, 0, 0}), kAbsError)
    COMPARE_ARRAY(msgData.nodes[1].pos_3d, ({2.435f, 2.399f, -1.117f}),
                  kAbsError)
    COMPARE_ARRAY(msgData.nodes[1].dis_arr,
                  ({3.18f, 2.98f, 5.3f, 5.31f, 0, 0, 0, 0}), kAbsError)
    EXPECT_EQ(msgData.local_time, 33313);
    EXPECT_EQ(msgData.system_time, 32000);
    EXPECT_NEAR(msgData.voltage, 4.995f, kAbsError);
  }
  {
    auto string =
        "55 01 01 02 8e 0a 00 a5 ff ff e8 03 00 da ff ff fa ff ff 00 00 00 "
        "35 0c 00 a3 15 00 cd 1a 00 4c 12 00 00 00 00 00 00 00 00 00 00 00 "
        "00 00 27 ac e2 3c a2 7d 0b 3c d2 70 3b bd cf a5 80 3e 3e fc 1b 41 "
        "1f a1 26 bd 26 5d 57 41 bd 80 57 41 3f 63 57 41 71 38 f5 25 44 fa "
        "8a 22 28 bf 5a b7 00 be 20 4f 3d bf 1c 0b 52 3d f4 26 3d 40 0c ae "
        "00 00 cb 17 01 00 f0 0b 10 ff 54 13 1d 48 00 00 bc fd";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msgData = linktrack::g_msg_tagframe0;
    COMPARE_ARRAY(msgData.pos_3d, ({2.702f, -0.091f, 1}), kAbsError)
    COMPARE_ARRAY(msgData.vel_3d, ({-0.0038f, -0.0006f, 0}), 0.0001f)
    COMPARE_ARRAY(msgData.dis_arr,
                  ({3.125f, 5.539f, 6.861f, 4.684f, 0, 0, 0, 0}), kAbsError)
    COMPARE_ARRAY(msgData.imu_gyro_3d, ({0.02767f, 0.008514f, -0.045762f}),
                  0.00001f)
    COMPARE_ARRAY(msgData.imu_acc_3d, ({0.251265f, 9.74908f, -0.040681f}),
                  0.00001f)
    COMPARE_ARRAY(msgData.angle_3d, ({144.49f, 97.17f, -14.68f}), 0.01f)
    COMPARE_ARRAY(msgData.quaternion,
                  ({-0.656777f, -0.125699f, -0.739489f, 0.0512801f}), 0.00001f)
    COMPARE_ARRAY(msgData.eop_3d, ({0.11f, 0.16f, 2.55f}), 0.01f)
    EXPECT_EQ(msgData.local_time, 44556);
    EXPECT_EQ(msgData.system_time, 71627);
    EXPECT_NEAR(msgData.voltage, 4.948f, kAbsError);
  }
  {
    auto string =
        "55 02 42 00 01 00 d1 2c c3 88 02 02 00 09 00 11 22 33 44 55 66 77 "
        "88 99 02 02 25 00 11 12 23 22 32 44 34 54 55 65 67 76 67 87 77 99 "
        "aa a2 13 45 57 65 56 56 56 56 57 78 43 33 34 44 44 44 44 46 76 0d";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msgData = linktrack::g_msg_nodeframe0;
    EXPECT_EQ(msgData.nodes[0].data.size(), 9);
    EXPECT_EQ(msgData.nodes[1].data.size(), 37);
  }
  {
    auto string =
        "55 03 44 00 03 00 e8 80 00 00 00 86 00 00 01 51 01 e8 01 f2 02 02 "
        "92 09 48 13 02 02 00 5f 0b 00 86 09 00 9b ff ff c9 37 8a 34 06 ee "
        "37 3f aa 02 02 93 09 00 45 09 00 c4 fc ff 8d 09 00 66 09 00 c4 fc "
        "ff 8e";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msgData = linktrack::g_msg_nodeframe1;
    COMPARE_ARRAY(msgData.nodes[0].pos_3d, ({2.911f, 2.438f, -0.101f}),
                  kAbsError)
    COMPARE_ARRAY(msgData.nodes[1].pos_3d, ({2.451f, 2.373f, -0.828f}),
                  kAbsError)
    EXPECT_NEAR(msgData.nodes[1].pos_3d[1], 2.373, 0.001);
    EXPECT_EQ(msgData.system_time, 33000);
    EXPECT_EQ(msgData.local_time, 34304);
    EXPECT_NEAR(msgData.voltage, 4.936f, kAbsError);
  }
  {
    auto string =
        "55 04 ac 00 02 01 ba 66 1d 00 06 09 ff de 0a 00 df ff ff e8 03 00 "
        "fa ff ff 1a 00 00 00 00 00 c0 12 00 00 00 00 00 00 00 27 ac e2 3c "
        "56 ed 1c 3c d2 70 3b bd 32 57 66 3e 3b cb 1b 41 93 70 61 bd 25 b2 "
        "6b 41 a1 22 6c 41 da da 6b 41 6d 23 e9 23 70 dd db f7 30 3f 5f d6 "
        "31 3f ba 81 1e 3e 47 69 e2 bd 91 9b 40 40 c5 23 00 00 40 40 5c d5 "
        "1c 00 00 00 1d 00 6d 13 04 01 00 6b 0c 00 b1 9f a6 66 1d 00 16 45 "
        "01 01 ac 15 00 b4 a1 a6 66 1d 00 d3 01 01 02 48 1a 00 ca 9f a6 66 "
        "1d 00 38 ba 01 03 2b 12 00 c6 a0 a6 66 1d 00 64 40 25";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msgData = linktrack::g_msg_nodeframe2;
    COMPARE_ARRAY(msgData.pos_3d, ({2.782f, -0.033f, 1}), kAbsError)
    COMPARE_ARRAY(msgData.vel_3d, ({-0.0006f, 0.0026f, 0}), 0.0001f)
    COMPARE_ARRAY(msgData.imu_gyro_3d, ({0.02767f, 0.00958f, -0.04576f}),
                  0.00001f)
    COMPARE_ARRAY(msgData.imu_acc_3d, ({0.224942f, 9.73712f, -0.05504f}),
                  0.00001f)
    COMPARE_ARRAY(msgData.angle_3d, ({90.69f, 91.93f, -88.48f}), 0.01f)
    COMPARE_ARRAY(msgData.quaternion,
                  ({0.691282f, 0.694677f, 0.154792f, -0.110552f}), 0.00001f)
    COMPARE_ARRAY(msgData.eop_3d, ({0.06f, 0.09f, 2.55f}), 0.01f)
    EXPECT_EQ(msgData.local_time, 9157);
    EXPECT_EQ(msgData.system_time, 1926842);
    EXPECT_NEAR(msgData.voltage, 4.973f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].dis, 3.179f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].fp_rssi, -88.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].rx_rssi, -79.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].dis, 5.548f, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].fp_rssi, -90, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].rx_rssi, -80.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].dis, 6.728f, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].fp_rssi, -101, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].rx_rssi, -79.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].dis, 4.651f, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].fp_rssi, -99, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].rx_rssi, -80, kAbsError);
  }
  {
    auto string =
        "55 05 32 00 02 01 f8 11 07 00 6f d0 6e 00 00 00 01 02 5a 13 04 01 "
        "00 22 0b 00 b5 9f 01 01 a3 17 00 b6 a0 01 02 88 1c 00 aa 9f 01 03 "
        "e6 14 00 b8 a0 9a";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msgData = linktrack::g_msg_nodeframe3;
    EXPECT_EQ(msgData.local_time, 463352);
    EXPECT_EQ(msgData.system_time, 7262319);
    EXPECT_NEAR(msgData.voltage, 4.954f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].dis, 2.85f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].fp_rssi, -90.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].rx_rssi, -79.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].dis, 6.051f, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].fp_rssi, -91, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].rx_rssi, -80, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].dis, 7.304f, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].fp_rssi, -85, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].rx_rssi, -79.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].dis, 5.35f, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].fp_rssi, -92, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].rx_rssi, -80, kAbsError);
  }
  {
    auto string =
        "55 06 40 00 01 03 8e 9d 01 00 8e 9d 01 00 ff ff 03 01 68 11 02 02 00 "
        "00 58 04 00 8d 09 00 01 d4 06 00 02 f6 07 00 03 ad 06 00 05 00 00 46 "
        "04 00 b8 0a 00 01 93 05 00 02 68 09 00 03 43 05 00 60 55 06 40 00 01 "
        "03 c0 9d 01 00 c0 9d 01 00 ff ff 03 01 68 11 02 02 00 00 59 04 00 7d "
        "09 00 01 d6 06 00 02 1f 08 00 03 0d 07 00 05 00 00 4d 04 00 d2 0a 00 "
        "01 80 05 00 02 64 09 00 03 32 05 00 3b 55 06 40 00 01 03 f2 9d 01 00 "
        "f2 9d 01 00 ff ff 03 01 58 11 02 02 00 00 59 04 00 6b 09 00 01 aa 06 "
        "00 02 e6 07 00 03 ed 06 00 05 00 00 4d 04 00 ab 0a 00 01 82 05 00 02 "
        "3d 09 00 03 45 05 00 bd 55 06 40 00 01 03 24 9e 01 00 24 9e 01 00 ff "
        "ff 03 01 58 11 02 02 00 00 59 04 00 76 09 00 01 c1 06 00 02 3b 08 00 "
        "03 e2 06 00 05 00 00 49 04 00 8d 0a 00 01 95 05 00 02 4a 09 00 03 32 "
        "05 00 7b";

    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msg_data = linktrack::g_msg_nodeframe4;
    EXPECT_EQ(msg_data.local_time, 106020);
    EXPECT_EQ(msg_data.system_time, 106020);
    EXPECT_NEAR(msg_data.voltage, 4.44f, kAbsError);
    {
      const auto &tag = msg_data.tags[0];
      EXPECT_EQ(tag.id, 2);
      EXPECT_NEAR(tag.voltage, 4.45f, kAbsError);
      EXPECT_EQ(tag.anchors.size(), 4);
      std::vector<std::pair<uint8_t, float>> datas = {
          {0, 2.422f},
          {1, 1.729f},
          {2, 2.107f},
          {3, 1.762f},
      };
      for (int i = 0; i < tag.anchors.size(); ++i)
      {
        const auto &anchor = tag.anchors[i];
        EXPECT_EQ(anchor.id, datas[i].first);
        EXPECT_NEAR(anchor.dis, datas[i].second, kAbsError);
      }
    }
    {
      const auto &tag = msg_data.tags[1];
      EXPECT_EQ(tag.id, 5);
      EXPECT_NEAR(tag.voltage, 3.65f, kAbsError);
      EXPECT_EQ(tag.anchors.size(), 4);
      std::vector<std::pair<uint8_t, float>> datas = {
          {0, 2.701f},
          {1, 1.429f},
          {2, 2.378f},
          {3, 1.33f},
      };
      for (int i = 0; i < tag.anchors.size(); ++i)
      {
        const auto &anchor = tag.anchors[i];
        EXPECT_EQ(anchor.id, datas[i].first);
        EXPECT_NEAR(anchor.dis, datas[i].second, kAbsError);
      }
    }
  }
  {
    auto string =
        "55 08 41 00 02 01 00 00 00 f8 11 07 00 6f d0 6e 00 00 00 01 02 5a 13 "
        "04 01 00 00 00 00 22 0b 00 b5 9f01 01 00 00 00 a3 17 00 b6 a0 01 02 "
        "00 00 00 88 1c 00 aa 9f 01 03 00 00 00 e6 14 00 b8 a0 ac";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msgData = linktrack::g_msg_nodeframe5;
    EXPECT_EQ(msgData.local_time, 463352);
    EXPECT_EQ(msgData.system_time, 7262319);
    EXPECT_NEAR(msgData.voltage, 4.954f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].dis, 2.85f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].fp_rssi, -90.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[0].rx_rssi, -79.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].dis, 6.051f, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].fp_rssi, -91, kAbsError);
    EXPECT_NEAR(msgData.nodes[1].rx_rssi, -80, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].dis, 7.304f, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].fp_rssi, -85, kAbsError);
    EXPECT_NEAR(msgData.nodes[2].rx_rssi, -79.5f, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].dis, 5.35f, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].fp_rssi, -92, kAbsError);
    EXPECT_NEAR(msgData.nodes[3].rx_rssi, -80, kAbsError);
  }

  {
    auto string = "55 09 4b 00 01 00 00 00 00 d1 2c c3 88 02 02 00 00 00 00 09 "
                  "00 11 22 33 44 55 66 77 88 99 02 02 00 00 00 25 00 11 12 23 "
                  "22 32 44 34 54 55 65 67 76 67 87 77 99 aa a2 13 45 57 65 56 "
                  "56 56 56 57 78 43 33 34 44 44 44 44 46 76 1d";
    auto data_length = NLink_StringToHex(string, data);
    protocol_extraction.AddNewData(data, data_length);
    auto &msgData = linktrack::g_msg_nodeframe6;
    EXPECT_EQ(msgData.nodes[0].data.size(), 9);
    EXPECT_EQ(msgData.nodes[1].data.size(), 37);
  }
}

namespace tofsense
{
  extern nlink_parser::TofsenseFrame0 g_msg_frame0;
}

TEST(NLinkParser, tofsense)
{
  NProtocolExtracter protocol_extraction;
  tofsense::Init init(&protocol_extraction, nullptr);

  uint8_t data[1024];
  auto string = "57 00 ff 00 c2 45 00 00 80 02 00 00 08 00 ff e6";
  auto data_length = NLink_StringToHex(string, data);
  protocol_extraction.AddNewData(data, data_length);

  auto &msg = tofsense::g_msg_frame0;
  EXPECT_EQ(msg.id, 0);
  EXPECT_EQ(msg.system_time, 17858);
  EXPECT_NEAR(msg.dis, 0.64, kAbsError);
  EXPECT_EQ(msg.dis_status, 0);
  EXPECT_EQ(msg.signal_strength, 8);
  EXPECT_EQ(msg.range_precision, 255);
}

namespace tofsensem
{
  extern nlink_parser::TofsenseMFrame0 g_msg_tofmframe0;
}

TEST(NLinkParser, tofsensem)
{
  NProtocolExtracter protocol_extraction;
  tofsensem::Init init(&protocol_extraction);

  uint8_t data[1024];
  auto string =
      "57 01 ff 00 13 c9 00 00 40 40 36 11 05 2d 00 30 80 13 05 16 00 c0 62 14 "
      "ff 09 00 98 21 19 ff 04 00 28 76 07 ff 08 00 38 20 07 ff 06 00 00 59 06 "
      "ff 08 00 c8 c0 12 ff 1f 00 f8 3b 12 05 27 00 a8 e3 14 05 1c 00 58 08 18 "
      "05 15 00 d8 34 1b 05 09 00 58 43 23 ff 0a 00 18 18 25 04 04 00 e0 04 07 "
      "ff 06 00 90 41 06 ff 07 00 48 7c 13 05 25 00 98 33 16 05 15 00 78 cd 19 "
      "05 0c 00 f0 9b 1e ff 09 00 e0 c1 26 ff 0b 00 80 d7 25 05 08 00 a0 31 24 "
      "05 09 00 38 78 22 05 06 00 98 bc 14 05 1b 00 48 5e 18 05 13 00 c8 84 1c "
      "05 08 00 08 68 26 05 07 00 58 b4 25 05 0b 00 80 dd 24 05 08 00 90 0a 24 "
      "05 0d 00 e8 56 07 05 16 00 f0 d1 15 05 1b 00 18 dd 19 05 0e 00 e8 37 21 "
      "04 07 00 d0 23 25 05 08 00 b0 52 25 05 08 00 70 39 24 05 0e 00 80 e9 22 "
      "05 0c 00 d0 5a 07 05 31 00 c8 a8 16 05 0d 00 18 54 1b 05 0c 00 60 18 23 "
      "0a 0a 00 f0 77 24 05 08 00 10 cc 23 05 0c 00 98 62 23 05 0f 00 f0 06 22 "
      "05 11 00 a8 37 07 05 46 00 b8 69 1a 05 0b 00 30 c1 1d 0a 0d 00 68 64 24 "
      "ff 0d 00 b0 58 24 05 08 00 28 4b 23 05 0d 00 b0 e7 21 05 12 00 38 01 21 "
      "05 0f 00 c8 08 07 05 44 00 18 dd 19 09 0a 00 38 6c 24 05 08 00 58 c0 23 "
      "05 0b 00 90 10 23 05 0c 00 a0 c0 21 05 0c 00 b8 3f 21 05 0c 00 a0 49 20 "
      "05 10 00 e0 04 07 05 32 00 ff ff ff ff ff ff 67";
  std::vector<float> vals{
      0,   51475, 1128, 5,   45, 1278, 5,   22, 1336, 255, 9,  1647,
      255, 4,     489,  255, 8,  467,  255, 6,  416,  255, 8,  1229,
      255, 31,    1195, 5,   39, 1369, 5,   28, 1575, 5,   21, 1783,
      5,   9,     2311, 255, 10, 2431, 4,   4,  460,  255, 6,  410,
      255, 7,     1277, 5,   37, 1455, 5,   21, 1691, 5,   12, 2006,
      255, 9,     2540, 255, 11, 2480, 5,   8,  2372, 5,   9,  2259,
      5,   6,     1359, 5,   27, 1597, 5,   19, 1869, 5,   8,  2517,
      5,   7,     2471, 5,   11, 2416, 5,   8,  2362, 5,   13, 481,
      5,   22,    1430, 5,   27, 1695, 5,   14, 2177, 4,   7,  2434,
      5,   8,     2446, 5,   8,  2374, 5,   14, 2288, 5,   12, 482,
      5,   49,    1485, 5,   13, 1791, 5,   12, 2300, 10,  10, 2390,
      5,   8,     2346, 5,   12, 2319, 5,   15, 2230, 5,   17, 473,
      5,   70,    1731, 5,   11, 1950, 10,  13, 2385, 255, 13, 2382,
      5,   8,     2313, 5,   13, 2222, 5,   18, 2163, 5,   15, 461,
      5,   68,    1695, 9,   10, 2387, 5,   8,  2343, 5,   11, 2298,
      5,   12,    2212, 5,   12, 2179, 5,   12, 2116, 5,   16, 460,
      5,   50};
  int val_cnt = -1;
  auto next_val = [&vals, &val_cnt]() { return vals[++val_cnt]; };
  auto data_length = NLink_StringToHex(string, data);
  protocol_extraction.AddNewData(data, data_length);

  auto &msg = tofsensem::g_msg_tofmframe0;
  EXPECT_EQ(msg.id, next_val());
  EXPECT_EQ(msg.system_time, next_val());
  for (int i = 0; i < msg.pixels.size(); ++i)
  {
    const auto &pixel = msg.pixels.at(i);
    EXPECT_NEAR(pixel.dis, next_val(), kAbsError);
    EXPECT_NEAR(pixel.dis_status, next_val(), kAbsError);
    EXPECT_NEAR(pixel.signal_strength, next_val(), kAbsError);
  }
}

namespace iot
{
  extern nlink_parser::IotFrame0 g_msg_iotframe0;
}

TEST(NLinkParser, iot)
{
  NProtocolExtracter protocol_extraction;
  iot::Init init(&protocol_extraction);

  uint8_t data[1024];
  auto string =
      "55 01 52 00 6b 00 4a 00 62 00 03 60 00 00 2b ef ff 00 00 00 47 00 48 00 "
      "ac 52 00 00 6e da 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00 00 00 00 00 00 00 "
      "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10";
  std::vector<float> vals{
      7012434, 6422602, 3, 0.096, -4.309, 0, 4718663, 172, 0.082, 55.918, 0,
      0,       0,       0, 0,     0,      0, 0,       0,   0,     0,      0,
      0,       0,       0, 0,     0,      0, 0,       0,   0,     0,      0,
      0,       0,       0, 0,     0,      0, 0,       0,   0,     0,      0,
      0,       0,       0, 0,     0,      0, 0};
  int val_cnt = -1;
  auto next_val = [&vals, &val_cnt]() { return vals[++val_cnt]; };
  auto data_length = NLink_StringToHex(string, data);
  protocol_extraction.AddNewData(data, data_length);

  auto &msg = iot::g_msg_iotframe0;
  EXPECT_NEAR(msg.uid, next_val(), kAbsError);
  for (int i = 0; i < msg.nodes.size(); ++i)
  {
    const auto &node = msg.nodes.at(i);
    EXPECT_NEAR(node.uid, next_val(), kAbsError);
    EXPECT_NEAR(node.cnt, next_val(), kAbsError);
    EXPECT_NEAR(node.dis, next_val(), kAbsError);
    EXPECT_NEAR(node.aoa_angle_horizontal, next_val(), kAbsError);
    EXPECT_NEAR(node.aoa_angle_vertical, next_val(), kAbsError);
  }
}

namespace linktrack_aoa
{
  extern nlink_parser::LinktrackAoaNodeframe0 g_msg_aoa_nodeframe0;
}

TEST(nlink_parser, linktrack_aoa)
{
  NProtocolExtracter protocol_extraction;
  linktrack_aoa::Init init(&protocol_extraction, nullptr);

  uint8_t data[1024];
  auto string =
      "55 07 42 00 02 00 be 73 02 00 00 00 00 00 00 00 f1 06 ef 12 04 01 00 ff "
      "02 00 22 0b a3 9f 9e 00 01 01 02 03 00 ad 00 a4 9f 00 00 01 02 ec 03 00 "
      "cb 03 a5 a0 00 00 01 03 88 05 00 99 ec a3 a0 00 00 33 ";
  auto data_length = NLink_StringToHex(string, data);
  protocol_extraction.AddNewData(data, data_length);

  auto &msg = linktrack_aoa::g_msg_aoa_nodeframe0;

  EXPECT_EQ(msg.role, LINKTRACK_ROLE_TAG);
  EXPECT_EQ(msg.id, 0);
  EXPECT_EQ(msg.local_time, 160702);
  EXPECT_EQ(msg.system_time, 0);
  EXPECT_NEAR(msg.voltage, 4.847f, kAbsError);
  EXPECT_EQ(msg.nodes.size(), 4);

  EXPECT_NEAR(msg.nodes[0].dis, 0.767f, kAbsError);
  EXPECT_NEAR(msg.nodes[0].angle, 28.50f, kAbsError);

  EXPECT_NEAR(msg.nodes[1].dis, 0.770f, kAbsError);
  EXPECT_NEAR(msg.nodes[1].angle, 1.73f, kAbsError);

  EXPECT_NEAR(msg.nodes[2].dis, 1.004f, kAbsError);
  EXPECT_NEAR(msg.nodes[2].angle, 9.71f, kAbsError);

  EXPECT_NEAR(msg.nodes[3].dis, 1.416f, kAbsError);
  EXPECT_NEAR(msg.nodes[3].angle, -49.67f, kAbsError);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_nlink_parser");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

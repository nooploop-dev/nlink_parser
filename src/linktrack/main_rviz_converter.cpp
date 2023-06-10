#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackNodeframe1.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include <ros/ros.h>

#include <map>
#include <sstream>

#include "nutils.h"

namespace {
std::string frameId;

struct PosePair {
  ros::Publisher publisher;
  geometry_msgs::PoseStamped msg;
  inline void publish() { publisher.publish(msg); }
};
} // namespace

void Anchorframe0Callback(const nlink_parser::LinktrackAnchorframe0 &msg) {
  static ros::Publisher publisher;
  static std::map<uint8_t, PosePair> poses;
  for (const auto &node : msg.nodes) {
    auto id = node.id;

    if (!poses.count(id)) {
      std::ostringstream string_stream;
      string_stream << "nlt_anchorframe0_pose_node" << static_cast<int>(id);
      auto topic = string_stream.str();
      poses[id].publisher =
          ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
      TopicAdvertisedTip(topic.c_str());
      auto &msg_pose = poses[id].msg;
      msg_pose.header.frame_id = frameId;
      msg_pose.pose.orientation.w = 0;
      msg_pose.pose.orientation.x = 0;
      msg_pose.pose.orientation.y = 0;
      msg_pose.pose.orientation.z = 1;
    }
    auto &msg_pose = poses[id].msg;
    ++msg_pose.header.seq;
    msg_pose.header.stamp = ros::Time(); // ros::Time(msg.system_time / 1000.0)
    msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
    poses.at(id).publish();
  }
}

void Nodeframe1Callback(const nlink_parser::LinktrackNodeframe1 &msg) {
  static ros::Publisher publisher;
  static std::map<uint8_t, PosePair> poses;
  for (const auto &node : msg.nodes) {
    auto id = node.id;

    if (!poses.count(id)) {
      std::ostringstream string_stream;
      string_stream << "nlt_nodeframe1_pose_node" << static_cast<int>(id);
      auto topic = string_stream.str();
      poses[id].publisher =
          ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
      TopicAdvertisedTip(topic.c_str());
      auto &msg_pose = poses[id].msg;
      msg_pose.header.frame_id = frameId;
      msg_pose.pose.orientation.w = 0;
      msg_pose.pose.orientation.x = 0;
      msg_pose.pose.orientation.y = 0;
      msg_pose.pose.orientation.z = 1;
    }
    auto &msg_pose = poses[id].msg;
    ++msg_pose.header.seq;
    msg_pose.header.stamp = ros::Time(); // ros::Time(msg.system_time / 1000.0)
    msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
    poses.at(id).publish();
  }
}

void Tagframe0Callback(const nlink_parser::LinktrackTagframe0 &msg) {
  static PosePair *pose = nullptr;
  if (!pose) {
    pose = new PosePair;
    auto topic = "nlt_tagframe0_pose";
    pose->publisher =
        ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
    TopicAdvertisedTip(topic);
    pose->msg.header.frame_id = frameId;
  }
  auto &msg_pose = pose->msg;
  ++msg_pose.header.seq;
  msg_pose.header.stamp = ros::Time(); // ros::Time(msg.system_time / 1000.0)
  msg_pose.pose.orientation.w = static_cast<double>(msg.quaternion[0]);
  msg_pose.pose.orientation.x = static_cast<double>(msg.quaternion[1]);
  msg_pose.pose.orientation.y = static_cast<double>(msg.quaternion[2]);
  msg_pose.pose.orientation.z = static_cast<double>(msg.quaternion[3]);
  msg_pose.pose.position.x = static_cast<double>(msg.pos_3d[0]);
  msg_pose.pose.position.y = static_cast<double>(msg.pos_3d[1]);
  msg_pose.pose.position.z = static_cast<double>(msg.pos_3d[2]);
  pose->publish();
}

void Nodeframe2Callback(const nlink_parser::LinktrackNodeframe2 &msg) {
  static PosePair *pose = nullptr;
  if (!pose) {
    pose = new PosePair;
    auto topic = "nlt_nodeframe2_pose";
    pose->publisher =
        ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
    TopicAdvertisedTip(topic);
    pose->msg.header.frame_id = frameId;
  }
  auto &msg_pose = pose->msg;
  ++msg_pose.header.seq;
  msg_pose.header.stamp = ros::Time(); // ros::Time(msg.system_time / 1000.0)
  msg_pose.pose.orientation.w = static_cast<double>(msg.quaternion[0]);
  msg_pose.pose.orientation.x = static_cast<double>(msg.quaternion[1]);
  msg_pose.pose.orientation.y = static_cast<double>(msg.quaternion[2]);
  msg_pose.pose.orientation.z = static_cast<double>(msg.quaternion[3]);
  msg_pose.pose.position.x = static_cast<double>(msg.pos_3d[0]);
  msg_pose.pose.position.y = static_cast<double>(msg.pos_3d[1]);
  msg_pose.pose.position.z = static_cast<double>(msg.pos_3d[2]);
  pose->publish();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_example");
  ros::NodeHandle nh;

  frameId = ros::param::param<std::string>("~map_frame", "linktrack_map");

  auto sub_anchorframe0 =
      nh.subscribe("nlink_linktrack_anchorframe0", 10, Anchorframe0Callback);

  auto sub_tagframe0 =
      nh.subscribe("nlink_linktrack_tagframe0", 10, Tagframe0Callback);

  auto sub_nodeframe1 =
      nh.subscribe("nlink_linktrack_nodeframe1", 10, Nodeframe1Callback);

  auto sub_nodeframe2 =
      nh.subscribe("nlink_linktrack_nodeframe2", 10, Nodeframe2Callback);

  ros::spin();

  return 0;
}

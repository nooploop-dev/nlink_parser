#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <nav_msgs/Path.h>
#include <nlink_parser/LinktrackAnchorframe0.h>
#include <nlink_parser/LinktrackNodeframe1.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackTagframe0.h>
#include <nutils.h>
#include <ros/ros.h>
#include <sstream>

namespace {
std::string frameId;

struct PosePair {
  ros::Publisher publisher;
  geometry_msgs::PoseStamped msg;
  inline void publish() { publisher.publish(msg); }
};
}

void anchorframe0Callback(const nlink_parser::LinktrackAnchorframe0 &msg) {
  static ros::Publisher publisher;
  static std::map<uint8_t, PosePair> poses;
  for (const auto &node : msg.tag) {
    auto id = node.id;

    if (!poses.count(id)) {
      std::ostringstream stringStream;
      stringStream << "nlt_anchorframe0_pose_node" << static_cast<int>(id);
      auto topic = stringStream.str();
      poses[id].publisher =
          ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
      topicadvertisedTip(topic.c_str());
      auto &msgPose = poses[id].msg;
      msgPose.header.frame_id = frameId;
      msgPose.pose.orientation.w = 0;
      msgPose.pose.orientation.x = 0;
      msgPose.pose.orientation.y = 0;
      msgPose.pose.orientation.z = 1;
    }
    auto &msgPose = poses[id].msg;
    ++msgPose.header.seq;
    msgPose.header.stamp = ros::Time(); // ros::Time(msg.systemTime / 1000.0)
    msgPose.pose.position.x = static_cast<double>(node.pos[0]);
    msgPose.pose.position.y = static_cast<double>(node.pos[1]);
    msgPose.pose.position.z = static_cast<double>(node.pos[2]);
    poses.at(id).publish();
  }
}

void nodeframe1Callback(const nlink_parser::LinktrackNodeframe1 &msg) {
  static ros::Publisher publisher;
  static std::map<uint8_t, PosePair> poses;
  for (const auto &node : msg.node) {
    auto id = node.id;

    if (!poses.count(id)) {
      std::ostringstream stringStream;
      stringStream << "nlt_nodeframe1_pose_node" << static_cast<int>(id);
      auto topic = stringStream.str();
      poses[id].publisher =
          ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
      topicadvertisedTip(topic.c_str());
      auto &msgPose = poses[id].msg;
      msgPose.header.frame_id = frameId;
      msgPose.pose.orientation.w = 0;
      msgPose.pose.orientation.x = 0;
      msgPose.pose.orientation.y = 0;
      msgPose.pose.orientation.z = 1;
    }
    auto &msgPose = poses[id].msg;
    ++msgPose.header.seq;
    msgPose.header.stamp = ros::Time(); // ros::Time(msg.systemTime / 1000.0)
    msgPose.pose.position.x = static_cast<double>(node.pos[0]);
    msgPose.pose.position.y = static_cast<double>(node.pos[1]);
    msgPose.pose.position.z = static_cast<double>(node.pos[2]);
    poses.at(id).publish();
  }
}

void tagframe0Callback(const nlink_parser::LinktrackTagframe0 &msg) {
  static PosePair *pose = nullptr;
  if (!pose) {
    pose = new PosePair;
    auto topic = "nlt_tagframe0_pose";
    pose->publisher =
        ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
    topicadvertisedTip(topic);
    pose->msg.header.frame_id = frameId;
  }
  auto &msgPose = pose->msg;
  ++msgPose.header.seq;
  msgPose.header.stamp = ros::Time(); // ros::Time(msg.systemTime / 1000.0)
  msgPose.pose.orientation.w = static_cast<double>(msg.q[0]);
  msgPose.pose.orientation.x = static_cast<double>(msg.q[1]);
  msgPose.pose.orientation.y = static_cast<double>(msg.q[2]);
  msgPose.pose.orientation.z = static_cast<double>(msg.q[3]);
  msgPose.pose.position.x = static_cast<double>(msg.pos[0]);
  msgPose.pose.position.y = static_cast<double>(msg.pos[1]);
  msgPose.pose.position.z = static_cast<double>(msg.pos[2]);
  pose->publish();
}

void nodeframe2Callback(const nlink_parser::LinktrackNodeframe2 &msg) {
  static PosePair *pose = nullptr;
  if (!pose) {
    pose = new PosePair;
    auto topic = "nlt_nodeframe2_pose";
    pose->publisher =
        ros::NodeHandle().advertise<geometry_msgs::PoseStamped>(topic, 10);
    topicadvertisedTip(topic);
    pose->msg.header.frame_id = frameId;
  }
  auto &msgPose = pose->msg;
  ++msgPose.header.seq;
  msgPose.header.stamp = ros::Time(); // ros::Time(msg.systemTime / 1000.0)
  msgPose.pose.orientation.w = static_cast<double>(msg.q[0]);
  msgPose.pose.orientation.x = static_cast<double>(msg.q[1]);
  msgPose.pose.orientation.y = static_cast<double>(msg.q[2]);
  msgPose.pose.orientation.z = static_cast<double>(msg.q[3]);
  msgPose.pose.position.x = static_cast<double>(msg.pos[0]);
  msgPose.pose.position.y = static_cast<double>(msg.pos[1]);
  msgPose.pose.position.z = static_cast<double>(msg.pos[2]);
  pose->publish();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_example");
  ros::NodeHandle nh;

  frameId = ros::param::param<std::string>("~map_frame", "/linktrack_map");

  auto subAnchorframe0 =
      nh.subscribe("nlink_linktrack_anchorframe0", 10, anchorframe0Callback);

  auto subTagframe0 =
      nh.subscribe("nlink_linktrack_tagframe0", 10, tagframe0Callback);

  auto subNodeframe1 =
      nh.subscribe("nlink_linktrack_nodeframe1", 10, nodeframe1Callback);

  auto subNodeframe2 =
      nh.subscribe("nlink_linktrack_nodeframe2", 10, nodeframe2Callback);

  ros::spin();

  return 0;
}

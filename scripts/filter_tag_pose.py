#!/usr/bin/env python

import sys
sys.path.append("./")
from one_euro_filter import OneEuroFilter

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nlink_parser.msg import LinktrackNodeframe2


def listener(pub, pub1):
    rospy.Subscriber("/nlink_linktrack_nodeframe2", LinktrackNodeframe2, callback, (pub, pub1))
    rospy.spin()


def callback(data, (pub, pub1)):
    rospy.loginfo("Tag Pose: %s", data.pos_3d)
    # define the data variable
    tag = data
    tag_filtered = LinktrackNodeframe2()
    tag_rviz = PoseStamped()
    
    # filter the tag pose
    tag_filtered = dataFilter(tag)
    # convert the tag pose to PoseStamped
    tag_rviz = msgToPoseStamped(tag_filtered)
    
    # publish the filtered tag pose and rviz PoseStamped
    publisher(pub, tag_filtered, pub1, tag_rviz)


def dataFilter(data):
    data_filtered = data
    filter = OneEuroFilter(freq=50, mincutoff=30, beta=0.01)
    
    pose_temp = list()
    for idx in range(3):
        pose_temp.append(filter(data.pos_3d[idx]))
    data_filtered.pos_3d = tuple(pose_temp)
    
    return data_filtered


def msgToPoseStamped(msg):
    pose = PoseStamped()
    
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "/linktrack_map"
    
    pose.pose.position.x = msg.pos_3d[0]
    pose.pose.position.y = msg.pos_3d[1]
    pose.pose.position.z = msg.pos_3d[2]
    
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    
    return pose


def publisher(pub, data, pub1, data1):
    rospy.loginfo("Filtered Tag Pose: %s", data.pos_3d)
    pub.publish(data)
    pub1.publish(data1)


def mainFunc():
    rospy.init_node('TagPoseFilter', anonymous=True)
    rospy.loginfo("Bingo, let's strart to receive, process and publish tag pose.")
    tag_pub = rospy.Publisher('/nlink_linktrack_nodeframe2_filtered', LinktrackNodeframe2)
    tag_pub1 = rospy.Publisher('/nlink_linktrack_nodeframe2_filtered_rviz', PoseStamped)
    listener(tag_pub, tag_pub1)


if __name__ == '__main__':
    mainFunc()
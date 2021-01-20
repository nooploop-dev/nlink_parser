#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import String


def mainFunc():
    rospy.init_node('DataTransmission', anonymous=True)
    rospy.loginfo("Here we go.")
    pub = rospy.Publisher('nlink_linktrack_data_transmission', String, queue_size=10)
    
    rate = rospy.Rate(20) # 50hz
    data = "540001ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffd9"
    hexcode = data.decode("hex")
    
    while not rospy.is_shutdown():
        rospy.loginfo(data)
        pub.publish(hexcode)
        rate.sleep()


if __name__ == '__main__':
    mainFunc()

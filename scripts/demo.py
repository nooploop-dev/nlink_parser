#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys, os, signal, time
from threading import Thread
sys.path.append("./")
from one_euro_filter import OneEuroFilter

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nlink_parser.msg import LinktrackNodeframe2, LinktrackSettingframe0


global is_exit
is_exit = False


class ThreadWatcher():
    """this class solves two problems with multithreaded 
    programs in Python, (1) a signal might be delivered 
    to any thread (which is just a malfeature) and (2) if 
    the thread that gets the signal is waiting, the signal 
    is ignored (which is a bug). 
    
    The watcher is a concurrent process (not thread) that 
    waits for a signal and the process that contains the 
    threads.  See Appendix A of The Little Book of Semaphores. 
    http://greenteapress.com/semaphores/ 
    
    I have only tested this on Linux.  I would expect it to 
    work on the Macintosh and not work on Windows. 
    """
    def __init__(self):  
        """ Creates a child thread, which returns.  The parent 
            thread waits for a KeyboardInterrupt and then kills 
            the child thread. 
        """  
        self.child = os.fork()  
        if self.child == 0:  
            return  
        else:  
            self.watch()
    
    def watch(self):  
        try:  
            os.wait()  
        except KeyboardInterrupt:  
            # I put the capital B in KeyBoardInterrupt so I can  
            # tell when the Watcher gets the SIGINT  
            print('KeyBoardInterrupt')
            self.kill()  
        sys.exit()
    
    def kill(self):  
        try:  
            os.kill(self.child, signal.SIGKILL)  
        except OSError:
            pass


def sendAnchorCmd(freq=30):
    global is_exit
    while not is_exit:
        rospy.loginfo("Sending request commands to serial for anchor poses.")
        pub_anchor_cmd = rospy.Publisher('nlink_linktrack_data_transmission', String, queue_size=10)
        
        rate = rospy.Rate(freq) # 50hz for default
        data = "540001ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffd9"
        hexcode = data.decode("hex")
        
        while not rospy.is_shutdown():
            pub_anchor_cmd.publish(hexcode)
            rate.sleep()


def getAnchorPose(num_anchor=4):
    global is_exit
    while not is_exit:
        rospy.loginfo("Getting anchor poses from serial.")
        pub_anchor_pose = list()
        for idx in range(num_anchor):
            pub_anchor_pose.append(rospy.Publisher('nlink_linktrack_anchor{}_pose_rviz'.format(idx), PoseStamped, queue_size=10))
        
        rospy.Subscriber("nlink_linktrack_settingframe0", LinktrackSettingframe0, anchorPoseCallback, pub_anchor_pose)
        rospy.spin()


def anchorPoseCallback(data, pub):
    anchor_pose = list()  # for ros msgs
    
    for idx in range(len(pub)):
        anchor_pose.append(msgToAnchorPose(data, idx))
        pub[idx].publish(anchor_pose[idx])


def msgToAnchorPose(msg, index):
    pose = PoseStamped()
    
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "/linktrack_map"
    
    pose.pose.position.x = msg.nodes[index].ag_3d[0]
    pose.pose.position.y = msg.nodes[index].ag_3d[1]
    pose.pose.position.z = msg.nodes[index].ag_3d[2]
    
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    
    return pose


def filterTagPose():
    global is_exit
    while not is_exit:
        rospy.loginfo("Bingo, let's strart to receive, process and publish tag pose.")
        pub_tag_pose = rospy.Publisher('nlink_linktrack_tag_pose_filtered_rviz', PoseStamped, queue_size=10)
        
        rospy.Subscriber("nlink_linktrack_nodeframe2", LinktrackNodeframe2, tagPoseCallback, pub_tag_pose)
        rospy.spin()


def tagPoseCallback(data, pub):
    tag_pose = LinktrackNodeframe2()  # for filtered data
    tag_pose_filtered = PoseStamped()  # for ros msgs
    
    tag_pose = dataFilter(data)
    tag_pose_filtered = msgToTagPose(tag_pose)
    
    pub.publish(tag_pose_filtered)


def dataFilter(data):
    data_filtered = data
    filter = OneEuroFilter(freq=50, mincutoff=40, beta=0.01)
    
    pose_temp = list()
    for idx in range(3):
        pose_temp.append(filter(data.pos_3d[idx]))
    data_filtered.pos_3d = tuple(pose_temp)
    
    return data_filtered


def msgToTagPose(msg):
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


def getTagPath():
    global is_exit
    while not is_exit:
        rospy.loginfo("Bingo, let's get and publish tag path.")
        pub_tag_path = rospy.Publisher('nlink_linktrack_tag_path_rviz', Path, queue_size=10)
        tag_path = Path() # for ros msgs of tag_path
        
        rospy.Subscriber("nlink_linktrack_tag_pose_filtered_rviz", PoseStamped, tagPathCallback, (pub_tag_path, tag_path))
        rospy.spin()


def tagPathCallback(data, (pub, path_var)):
    path_var = msgToTagPath(data, path_var)
    
    pub.publish(path_var)


def msgToTagPath(msg, path):
    # path.header.stamp = rospy.Time.now()
    # path.header.frame_id = "/linktrack_map"
    path.header = msg.header
    path.poses.append(msg)
    
    return path


def signalHandler(sig_num, frame):
    global is_exit
    is_exit = True
    print("Receive {} signal, is_exit = {} and quit.".format(sig_num, is_exit))
    sys.exit(0)


def mainFunc():
    rospy.init_node('Demo', anonymous=True)
    
    # start the watcher for stopping all threads
    # ThreadWatcher()
    
    # set the signal handler for stopping all threads
    signal.signal(signal.SIGINT, signalHandler)
    signal.signal(signal.SIGTERM, signalHandler)
    
    # set the thread list
    thread_name = ['sendAnchorCmd', 'getAnchorPose', 'filterTagPose', 'getTagPath']
    thread_list = list()
    
    # start all threads from thread_name list
    for idx in range(len(thread_name)):
        thread = Thread(target=eval(thread_name[idx]))
        thread.setDaemon(True)
        thread.start()
        thread_list.append(thread)
        
    # join all threads from thread_name list
    # for idx in range(len(thread_name)):
    #     thread_list[idx].join()
    
    # stop all subthreads if excepts happen
    while True:
        alive = False
        for i in range(len(thread_list)):
            alive = alive or thread_list[i].isAlive()
        if not alive:
            break


if __name__ == '__main__':
    mainFunc()

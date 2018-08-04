#!/usr/bin/env python
import rospy

import rospy
import yaml

import sys

from take_photo import TakePhoto
from go_to_specific_point_on_map import GoToPose

from std_msgs.msg import Bool
from std_msgs.msg import UInt32
from std_msgs.msg import String

from wall_follow import WallFollow

RIGHT = 'right'
LEFT  = 'left'

class Follower():
    def __init__(self):
        # Read information from yaml file
        with open("/home/robot/git/turtlebot_actions/turtlebot/route.yaml", 'r') as stream:
            self.dataMap = yaml.load(stream)

    def amcl_pose_callback(msg)
        
    def go(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz        
        
         while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()
        
if __name__ == '__main__':
     try:
         Follower()
     except rospy.ROSInterruptException:
         pass
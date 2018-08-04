#!/usr/bin/env python

import rospy
import yaml

import sys
import math

from take_photo import TakePhoto
from go_to_specific_point_on_map import GoToPose

from std_msgs.msg import Bool
from std_msgs.msg import UInt32
from std_msgs.msg import String

from wall_follow import WallFollow

RIGHT = 'right'
LEFT  = 'left'

old_x = 0.0
old_y = 0.0

distance = 0.0

wall_session = False

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def amcl_pose_callback(msg):
    print msg.pose.pose
    
def odometry_callback(msg):
    global old_x
    global old_y
    global distance
    
    new_x = msg.pose.pose.position.x
    new_y = msg.pose.pose.position.y

    if wall_session == False:
        old_x = new_x
        old_y = new_y 
        distance = 0.0
        
    if wall_session == True:
        p1 = (new_x, new_y)
        p2 = (old_x, old_y)
        distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

if __name__ == '__main__':

    with open("/home/robot/git/turtlebot_actions/turtlebot/route.yaml", 'r') as stream:
        dataMap = yaml.load(stream)
    try:
        rospy.init_node('follow_route', anonymous=False)
        
        global wall_session
        
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
        rospy.Subscriber("odom", Odometry, odometry_callback)
        
        start_session_pub = rospy.Publisher('photo/start_session', Bool, queue_size=10)
        session_state_msg = Bool()
        
        session_num_pub = rospy.Publisher('photo/session_num', UInt32, queue_size=10)
        session_num_msg = UInt32()
        
        rack_id_pub = rospy.Publisher('photo/rack_id', UInt32, queue_size=10)
        rack_id_msg = UInt32()

        navigator = GoToPose()
        camera = TakePhoto()
        wall = WallFollow(LEFT)
        wall.stop()

        for obj in dataMap:

            if rospy.is_shutdown():
                break

            name = obj['filename']
            
            session_num_msg = obj['session']
            rack_id_msg = obj['rackid']
            session_state_msg = True

            rospy.loginfo("Go to %s pose of rack %s", name[:-4], rack_id_msg)

            start_session_pub.publish(session_state_msg)
            session_num_pub.publish(session_num_msg)
            rack_id_pub.publish(rack_id_msg)

            if obj['rackid'] != 0:
                rospy.loginfo("Start wall")
                wall_session = True
                wall.start()
            
                while distance < 3.0:
                    rospy.loginfo("Distance is: %s", distance)
                    rospy.sleep(0.1)
                    
                rospy.loginfo("Stop wall")                
                wall.stop()
                wall_session = False
                
            else:
                rospy.loginfo("Start navi")
                success = navigator.goto(obj['position'], obj['quaternion'])

                rospy.loginfo("Stop navi")
                if not success:
                    rospy.loginfo("Failed to reach %s pose", name[:-4])
                    continue

            rospy.loginfo("Reached %s pose", name[:-4])

            # Take a photo from kinnect
            if camera.take_picture(name):
                rospy.loginfo("Saved image " + name)
            else:
                rospy.loginfo("No images received")
                
            session_state_msg = False
            start_session_pub.publish(session_state_msg)
            
            rospy.sleep(1)
            
        rospy.loginfo("Mission complete")
        rospy.signal_shutdown("End")

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
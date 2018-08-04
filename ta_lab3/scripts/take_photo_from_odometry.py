#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Bool
from std_msgs.msg import UInt32
from nav_msgs.msg import Odometry

import sys
import pycurl
import urllib
import json

from io import BytesIO

url = 'http://127.0.0.1:8088/photo'

session_state = False

photo_count = 0
session = 0
rackid = 0
    
old_x = 0.0
old_y = 0.0

def dumpclean(obj):
    if type(obj) == dict:
        for k, v in obj.items():
            if hasattr(v, '__iter__'):
                print k
                dumpclean(v)
            else:
                print '%s : %s' % (k, v)
    elif type(obj) == list:
        for v in obj:
            if hasattr(v, '__iter__'):
                dumpclean(v)
            else:
                print v
    else:
        print obj
        
def take_goods_photo(ses,rac,cnt):

    params = {'session': 1, 'rackid': 1, 'count': 1}
    data = BytesIO()

    c = pycurl.Curl()

    params["session"] = ses
    params["rackid"] = rac
    params["count"] = cnt

    c.setopt( pycurl.URL, url + '?' + urllib.urlencode(params) )
    c.setopt(pycurl.HTTPGET, True)
    c.setopt( pycurl.WRITEFUNCTION, data.write )
 
    c.perform()
    c.close()

    dictionary = json.loads(data.getvalue())

    #dumpclean(dictionary)

    return dictionary["status"]

def odometry_callback(msg):
    global old_x
    global old_y
    global photo_count
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",msg)
    
    new_x = msg.pose.pose.position.x
    new_y = msg.pose.pose.position.y

    #rospy.loginfo("Session state: %s", session_state)
    
    if session_state == False:
        photo_count = 0
        old_x = new_x
        old_y = new_y        
        
    if session_state == True:
        p1 = (new_x, new_y)
        p2 = (old_x, old_y)

        distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        #rospy.loginfo("Distance is: %s", distance)
        
        if distance >= 0.1:
            old_x = new_x
            old_y = new_y
            
            photo_count += 1

            rospy.loginfo("I toke photo number: %s", photo_count)
            
            # Take photo from cameras
            if take_goods_photo(session, rackid, photo_count) == 'ok':
                print 'all right'            

        
def state_callback(msg):
    global session_state     
    session_state = msg.data
    rospy.loginfo("I heard state: %s", session_state)
    
def rack_id_callback(msg):
    global rackid
    rackid = msg.data
    rospy.loginfo("I heard rackid: %s", rackid)
    
def session_num_callback(msg):
    global session
    session = msg.data
    rospy.loginfo("I heard session number: %s", session)
    
def photo():
   
    rospy.init_node('photo', anonymous=True)
    
    rospy.Subscriber("odom", Odometry, odometry_callback)
    #rospy.Subscriber()
    
    rospy.Subscriber("photo/session_num", UInt32, session_num_callback)
    rospy.Subscriber("photo/rack_id", UInt32, rack_id_callback)
    rospy.Subscriber("photo/start_session", Bool, state_callback)
    
    rospy.spin()

if __name__ == '__main__':
    photo()

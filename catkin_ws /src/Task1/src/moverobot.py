#! /usr/bin/env python

import rospy
# import twist messages (liner and angular speed(x,y,z)),
# our publisher publish these speed information to /cmd_vel topic 
from geometry_msgs.msg import Twist
# LaserScan messages(laser distances information)
# ou subscriber will take these information from topic /kobuki/laser/scan.
# depnding on the data we recieve from laser scan topic we change the messages in \cmd_vel(change x, y, z) 
from sensor_msgs.msg import LaserScan
import time 

class MoveRob:
    def __init__(self):
        rospy.init_node('MoveTurtle3') 
        self.pubNode = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.subNode = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.msgTwistObj = Twist()
        self.laserMsg = LaserScan()


    def callback(self, msg):
        self.laserMsg = msg
    
    def getLaserScanMsgDistance(self):
        if len(self.laserMsg.ranges) == 0:
            return 0
        return self.laserMsg.ranges[360]

    def moveRob(self):
        while not rospy.is_shutdown():
          
            self.pubNode.publish(self.msgTwistObj)
            
            while self.getLaserScanMsgDistance() > 1 :
                self.msgTwistObj.linear.x = 0.1
            
            self.msgTwistObj.linear.x = 0
            self.msgTwistObj.angular.z = 0.5
            time.sleep(0.9)
            self.msgTwistObj.linear.x = 0.5
            
        
obj1 = MoveRob()
obj1.moveRob()
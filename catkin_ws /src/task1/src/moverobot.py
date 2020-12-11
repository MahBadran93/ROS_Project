#! /usr/bin/env python

import rospy
# import twist messages (liner and angular speed(x,y,z)),
# our publisher publish these speed information to /cmd_vel topic 
from geometry_msgs.msg import Twist
# LaserScan messages(laser distances information)
# ou subscriber will take these information from topic /scan.
# depnding on the data we recieve from laser scan topic we change the messages in \cmd_vel(change x, y, z) 
from sensor_msgs.msg import LaserScan

from rosgraph_msgs.msg import Clock

import time 

class MoveRob:
    def __init__(self):
        rospy.init_node('MoveTurtle3') 
        # publish to cmd_vel topic to send velocity information to the robot 
        self.pubNode = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # subscribe to scan to take the laser information 
        self.subNode = rospy.Subscriber('/scan', LaserScan, self.callback)

        #Clock timer
        #self.clock_sub = rospy.Subscriber('/clock', Clock , self.event_clock)

        # init an object called msgTwistObj to hold the Twist msg information which will be 
        # carried by cmd_vel topic to send to the robot 
        self.msgTwistObj = Twist()

        # init an object called laserMsg to hold an information sent by the LaserScan information 
        # carried by scan topic 
        self.laserMsg = LaserScan()

        # Timer information 
        self.ref_time = -1
        self.current_time = -1
        self.counter = 0
        rospy.spin()

    # LaserScan Message callback
    def callback(self, msg):

        self.laserMsg = msg
        # If the distance between a robot and a wall is less than 1m, stop the Robot!
        if self.getLaserDistance() < 1:
            self.msgTwistObj.linear.x = 0.0
            self.msgTwistObj.angular.z = 0.0

        # Keep moving if the distance is larger than 1m.
        if self.getLaserDistance() > 1:
            self.msgTwistObj.linear.x = 0.2
            self.msgTwistObj.angular.z = 0.0  

        self.pubNode.publish(self.msgTwistObj) 

    # Ranges value, return the laser value at 90 degree(ranges[0]) to know if there is obstacles infront of the Robot
    def getLaserDistance(self):
        if len(self.laserMsg.ranges) == 0:
            return 0
        return self.laserMsg.ranges[0]

            
#..............................................................................

# Create an object of class MoveRob         
obj1 = MoveRob()
# Execute the MoveRob function 
#obj1.moveRob()
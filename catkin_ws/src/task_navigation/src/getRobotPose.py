#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse 
# import important messages for position 
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

robotPose = Pose()

def service_callback(posedata):
    print('Robot Position',robot_pose )
    return EmptyResponse() # the service Response class, in this case EmptyResponse
    
def sub_callback(msg):
    global robot_pose
    # assign the current Robot poition to robotPose parameter 
    # we obtain the position from amcl_pose topic as PoseWithCovarianceStamped msg  
    robotPose = msg.pose.pose
    

rospy.init_node('geRobotPose') 
my_service = rospy.Service('/get_pose_service', Empty , service_callback) # create the Service called get_pose_service with the defined callback
# get the robot estimated pose in the map
sub_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, sub_callback)
rospy.spin() # mantain the service open.
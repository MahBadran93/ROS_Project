#!/usr/bin/env python

# we use this node to send specific goal to move base by SimpleActionClient

import rospy
# import actionlib to access SimpleActionServer which will send goals to movebase
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # we obtain goal coords using rostopic echo /move_base_simple/goal, 
    # when  we press 2d nav goal we will see the coordinates in the terminal 
    # Goal Translation
    goal.target_pose.pose.position.x = -9.72207355499
    goal.target_pose.pose.position.y = 1.0002155304
    goal.target_pose.pose.position.z = 0
    # Goal Orientation
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.w = 0.704986317927
    goal.target_pose.pose.orientation.z = -0.70922090461

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('sendGoal')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
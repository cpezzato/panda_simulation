#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64MultiArray
from armGoalAction import activeInferenceAction
import active_inference_msgs.msg
import actionlib


rospy.init_node('arm_goal_client')

client = actionlib.SimpleActionClient('arm_goal_action', active_inference_msgs.msg.ActiveInferenceAction)
client.wait_for_server()

p = active_inference_msgs.msg.ActiveInferenceActionGoal()
rospy.loginfo(p)
p.goal.pose.position.x = 0.2
p.goal.pose.position.y = -0.422
p.goal.pose.position.z = 0.591
p.goal.pose.orientation.x = 0.713
p.goal.pose.orientation.y = 0.701
p.goal.pose.orientation.z = 0.005
p.goal.pose.orientation.w = 0.004


client.send_goal(p.goal)
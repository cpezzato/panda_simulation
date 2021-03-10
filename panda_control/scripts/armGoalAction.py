#!/usr/bin/env python


import rospy
import actionlib
from rospy.core import loginfo
from std_msgs.msg import Float64MultiArray 
import active_inference_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from mm_msgs.srv import MMIk
import numpy as np

class activeInferenceAction(object):
    _feedback = active_inference_msgs.msg.ActiveInferenceFeedback()
    _result = active_inference_msgs.msg.ActiveInferenceResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, active_inference_msgs.msg.ActiveInferenceAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._pub = rospy.Publisher("arm_goal", Float64MultiArray, queue_size=10)
        self._sub = rospy.Subscriber("/mmrobot/joint_states", JointState, self.jointStateCb)
        self._computeIk = rospy.ServiceProxy('ik_computation', MMIk)

        self._joint_state = JointState()
        self._goal_reached_threshold = 0.01

    def jointStateCb(self, msg):
        self._joint_state = msg.position[3:-2]
      
    def execute_cb(self, pose_goal):  
        # Convert cartesian pose to joint space      
        pose_stamped_goal = PoseStamped()
        pose_stamped_goal.pose = pose_goal.pose
        pose_stamped_goal.header.frame_id = "mmrobot_link0"
        joint_state_goal = self._computeIk(pose_stamped_goal)

        # rospy.loginfo("IK config : %s",  joint_state_goal.config.data)
        rospy.logwarn("IK return code: %s", joint_state_goal.errorFlag)

        # rospy.logwarn("in execute_cb, goal: %s", joint_state_goal)
        
        arm_goal = Float64MultiArray()
        arm_goal.data = joint_state_goal.config.data

        # send arm goal to AIC controller
        self._pub.publish(arm_goal)
        goal_reached = 0

        # send current pose if action preempted, goal pose if goal reached, set action goal as preempted or succeeded
        while not goal_reached:
            dist_from_goal = np.linalg.norm(np.array(self._joint_state) - np.array(arm_goal.data))

            if dist_from_goal < self._goal_reached_threshold:
                goal_reached = 1
                self._as.set_succeeded()
                break

            if self._as.is_preempt_requested():
                rospy.loginfo("%s preempted", self._action_name)
                arm_goal.data = self._joint_state
                self._pub.publish(arm_goal)
                self._as.set_preempted()
                self.success = False
                break

if __name__ == '__main__':
    rospy.init_node('arm_goal_action')
    server = activeInferenceAction(rospy.get_name())
    rospy.spin()
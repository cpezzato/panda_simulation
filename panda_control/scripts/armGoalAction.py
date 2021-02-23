#!/usr/bin/env python


import rospy
import actionlib
from std_msgs.msg import Float64MultiArray 
import active_inference_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from mm_msgs.srv import MMIk

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

    def jointStateCb(self, msg):
        self._joint_state = msg.position[3:-2]
      
    def execute_cb(self, pose_goal):        
        pose_stamped_goal = PoseStamped()
        pose_stamped_goal.pose = pose_goal.pose
        pose_stamped_goal.header.frame_id = "mmrobot_link0"
        joint_state_goal = self._computeIk(pose_stamped_goal)

        rospy.loginfo("config : %s",  joint_state_goal.config.data)
        rospy.logwarn("return code: %s", joint_state_goal.errorFlag)


        rospy.logwarn("in execute_cb, goal: %s", joint_state_goal)
        arm_goal = Float64MultiArray()

        arm_goal.data = joint_state_goal.config.data

        self._pub.publish(arm_goal)

        while not self._as.is_preempt_requested():
            pass # Just keep running until goal is canceled (once AIC is tuned we can finish goals when the arm reaches the required joint poses)

        rospy.logwarn('%s: Goal no longer active!!' % self._action_name)
        self._as.set_preempted()
        success = False
        arm_goal.data = self._joint_state
        self._pub.publish(arm_goal)

            




if __name__ == '__main__':
    rospy.init_node('arm_goal_action')
    server = activeInferenceAction(rospy.get_name())
    rospy.spin()
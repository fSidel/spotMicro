#!/usr/bin/python

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from spot_micro_perception.msg import Detection
from math import pi
import cv2


class SpotMicroPerceptionControl():
    node_name = "perception_control_node"
    detection_subscription = "detection_topic"

    def __init__(self):

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._vel_cmd_msg = Twist()
        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True  # Mostly acts as an event driven action on receipt of a true message

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True

        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Perception Control Node...")

        rospy.init_node(self.node_name,
                        anonymous=True)

        # Create publishers for commanding velocity, angle, and robot states
        self._ros_pub_angle_cmd = rospy.Publisher('/angle_cmd', 
                                                  Vector3, 
                                                  queue_size=1)
        
        self._ros_pub_vel_cmd = rospy.Publisher('/cmd_vel', 
                                                Twist, 
                                                queue_size=1)
        
        self._ros_pub_walk_cmd = rospy.Publisher('/walk_cmd', 
                                                 Bool, 
                                                 queue_size=1)
        self._ros_pub_stand_cmd = rospy.Publisher('/stand_cmd', 
                                                  Bool, 
                                                  queue_size=1)
        
        self._ros_pub_idle_cmd = rospy.Publisher('/idle_cmd', 
                                                 Bool, 
                                                 queue_size=1)

        rospy.loginfo("Perception control node publishers corrrectly initialized")


    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''
        
        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

    def reset_all_angle_commands_to_zero(self):
        '''Reset angle cmd states to zero and publish them'''

        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

    def run():
        pass


if __name__ == "__main__":
    smpc = SpotMicroPerceptionControl()
    smpc.run()

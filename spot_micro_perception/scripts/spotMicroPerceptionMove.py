#!/usr/bin/python

import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from spot_micro_perception.msg import Detections
from math import pi
import json
import cv2


class SpotMicroPerceptionControl():
    node_name = "perception_control_node"
    detection_subscription = "detection_topic"
    angle_publication = "/angle_cmd"
    velocity_publication = "/cmd_vel"
    walk_publication = "/walk_cmd"
    stand_publication = "/stand_cmd"
    idle_publication = "/idle_cmd"

    # Defines how the robot moves by changing its angle

    MAX_ROLL_DEG = 45   
    MAX_YAW_DEG = 45
    MAX_PATCH_DEG = 45      # For now, this is the only value we change to track the object on the y axis

    # Defines how the robot moves by moving its limbs

    MAX_FORWARD_SPEED = 0.05
    MAX_STRAFE_SPEED = 0.05     # For now, this is the only value we change to track the object on the x axis
    MAX_YAW_SPEED_DEG = 15

    
    def __init__(self):
        self.tracking_id = rospy.get_param('/perception_track/tracking_id')
        rospy.loginfo("Class to track set to: ")
        rospy.loginfo(self.tracking_id)
        
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
        self._walk_event_cmd_msg.data = True 

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True

        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Perception Control Node...")

        rospy.init_node(self.node_name,
                        anonymous=True)

        # Create publishers for commanding velocity, angle, and robot states
        self._ros_pub_angle_cmd = rospy.Publisher(self.angle_publication, 
                                                  Vector3, 
                                                  queue_size=1)
        
        self._ros_pub_vel_cmd = rospy.Publisher(self.velocity_publication, 
                                                Twist, 
                                                queue_size=1)
        
        self._ros_pub_walk_cmd = rospy.Publisher(self.walk_publication, 
                                                 Bool, 
                                                 queue_size=1)
        self._ros_pub_stand_cmd = rospy.Publisher(self.stand_publication, 
                                                  Bool, 
                                                  queue_size=1)
        
        self._ros_pub_idle_cmd = rospy.Publisher(self.idle_publication, 
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

    
    def detection_callback(self, message):
        detections = json.loads(message.detections)
    
        trackable_detections = list(filter(lambda detection: detection['id'] == self.tracking_id,
                                           detections))
        
        if not trackable_detections:
            rospy.loginfo("No detections found of the given class, idling.")
        else:
            #TODO: this is very naive, should implement better logic for choosing what detection to track
            first_detection = trackable_detections[0]
            rospy.loginfo("tracking detection: %s", first_detection)
            rospy.loginfo(first_detection['x'])
            rospy.loginfo(first_detection['y'])

            #TODO: retrieve max parameters for controlling the robot from spot_micro_motion_cmd/config/spot_micro_motion_cmd.yaml
        

    def on_detection_angle_mode(self, axes):
        self._angle_cmd_msg.x = pi / 180 * axes[self.ANGLE_AXES_ROLL] * self.MAX_ROLL_DEG * -1
        self._angle_cmd_msg.y = pi / 180 * axes[self.ANGLE_AXES_PITCH] * self.MAX_PATCH_DEG * -1
        self._angle_cmd_msg.z = pi / 180 * axes[self.ANGLE_AXES_YAW] * self.MAX_YAW_DEG
        print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg ' \
            % (
                self._angle_cmd_msg.x * 180 / pi, self._angle_cmd_msg.y * 180 / pi,
                self._angle_cmd_msg.z * 180 / pi))
        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
            
            

    def run(self):
        self.reset_all_motion_commands_to_zero()
        
        rospy.Subscriber(self.detection_subscription, 
                         Detections, 
                         self.detection_callback)

        rospy.spin()


if __name__ == "__main__":
    smpc = SpotMicroPerceptionControl()
    smpc.run()

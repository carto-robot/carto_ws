#!/usr/bin/env python3

import rospy
import sys
import os
import argparse
import time
import threading
import numpy as np

import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import geometry_msgs.msg
import tf_conversions

TIMEOUT_DURATION = 20.0 # seconds


class PIDController:
    def __init__(self, Kp, Ki, Kd, max_integral, max_output):
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        self.max_integral = np.array(max_integral)
        self.max_output = np.array(max_output)
        
        self.integral = np.zeros_like(self.Kp)
        self.prev_error = np.zeros_like(self.Kp)
        self.prev_current = np.zeros_like(self.Kp)
        self.prev_out = np.zeros_like(self.Kp)
        
    def compute(self, setpoint, current, dt):
        error = np.array(setpoint) - np.array(current)
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        I = self.Ki * self.integral
        
        # Derivative term with derivative filtering (differential pre-action)
        # derivative = (self.prev_current - current) / dt
        derivative = (self.prev_error - error) / dt
        derivative = np.clip(derivative, -self.max_integral*0.2, self.max_integral*0.2)
        D = self.Kd * derivative
        
        # PID output
        output = P + I + D
        
        # Save error and derivative for next iteration
        self.prev_error = error
        self.prev_current = np.array(current)
        
        # Apply output limits
        output = np.clip(output, -self.max_output, self.max_output)
        self.prev_out = output
        
        return output

def control_robot(target_pose, current_pose, max_speeds, dt, pid_controllers):
    # Extract positions and orientations
    target_pos = np.array(target_pose[:3])
    target_orient = np.array(target_pose[3:])
    current_pos = np.array(current_pose[:3])
    current_orient = np.array(current_pose[3:])
    
    # print(target_pos)
    # print(target_orient)
    # print(current_pos)
    # print(current_orient)
    # Calculate the velocity commands
    pos_velocity = pid_controllers[0].compute(target_pos, current_pos, dt)
    orient_velocity = pid_controllers[1].compute(target_orient, current_orient, dt)
    
    # # Combine velocities
    velocity = np.concatenate((pos_velocity, orient_velocity))
    max_speeds = np.array(max_speeds)
    # # Apply maximum speed limits
    velocity = np.clip(velocity, -max_speeds, max_speeds)
    
    return velocity



def transform_pose(input_pose, transform):
    """
    Transform a pose from one frame to another using a given transform.

    :param input_pose: PoseStamped in the original frame
    :param transform: TransformStamped to the target frame
    :return: PoseStamped in the target frame
    """
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    target_pose = tf2_geometry_msgs.do_transform_pose(input_pose, transform)
    return target_pose


def vision_servo_node():



    # Parse arguments from ROS parameter server
    tag_frame = rospy.get_param('~tag_frame', 'tag_3')

        
    rospy.init_node('vision_servo_node', anonymous=True)
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer)
    broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(50.0)  # 50 Hz


    while not rospy.is_shutdown():
        try:
            #获取二维码在end_effector_link坐标系下的位姿
            transform = tf_buffer.lookup_transform("wrist", tag_frame, rospy.Time(0))
            # rospy.loginfo(f"Received transform: {transform}")

            # Define the target_pose in the tag_frame
            _target_pose = geometry_msgs.msg.PoseStamped()
            _target_pose.header.frame_id = tag_frame
            _target_pose.pose.position.x = 0
            _target_pose.pose.position.y = 0
            _target_pose.pose.position.z = 0
            # (roll,pitch,yaw) ->(x,y,z,w)
            quat = tf_conversions.transformations.quaternion_from_euler(0, 1.57, 0)
            _target_pose.pose.orientation.x = quat[0]
            _target_pose.pose.orientation.y = quat[1]
            _target_pose.pose.orientation.z = quat[2]
            _target_pose.pose.orientation.w = quat[3]

            # Transform the target_pose to the end_effector_link frame
            target_pose = transform_pose(_target_pose, transform)

            #发布到tf树便于调试
            # Create a TransformStamped message
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "wrist"
            t.child_frame_id = "target_pose"
            t.transform.translation.x = target_pose.pose.position.x
            t.transform.translation.y = target_pose.pose.position.y
            t.transform.translation.z = target_pose.pose.position.z
            t.transform.rotation = target_pose.pose.orientation

            # Broadcast the transform
            broadcaster.sendTransform(t)

        except Exception as e:
            rospy.logerr(f"Failed to lookup transform: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        vision_servo_node()
    except rospy.ROSInterruptException:
        pass

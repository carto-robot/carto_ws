#!/usr/bin/env python3

import rospy
import sys
import os
import argparse
import time
import threading
import numpy as np

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import geometry_msgs.msg
import tf_conversions

TIMEOUT_DURATION = 20.0 # seconds

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def example_move_to_home_position(base):

    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(20)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def cartesian_action_movement(base,target_pose):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = target_pose[0]    # (meters)
    cartesian_pose.y = target_pose[1]    # (meters)
    cartesian_pose.z = target_pose[2]    # (meters)
    cartesian_pose.theta_x = target_pose[3] # (degrees)
    cartesian_pose.theta_y = target_pose[4] # (degrees)
    cartesian_pose.theta_z = target_pose[5] # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def twist_command(base,twist_command):

    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = twist_command[0]
    twist.linear_y = twist_command[1]
    twist.linear_z = twist_command[2]    
    twist.angular_x = twist_command[3]
    twist.angular_y = twist_command[4]
    twist.angular_z = twist_command[5]

    base.SendTwistCommand(command)

    return True

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


def arm_control_node():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "."))
    import utilities

    # 下面这段用法只能在rosrun的时候使用，不能在launch文件中使用，因为utilities.py文件是读取命令行参数的，
    # 而launch文件中启动节点时并没有命令行参数，所以这里需要用rospy.get_param()来获取参数
    # # Parse arguments
    # args = utilities.parseConnectionArguments()

    #下面这段用法可以在launch文件中使用
        # Parse arguments from ROS parameter server
    ip = rospy.get_param('~ip', '192.168.1.10')
    username = rospy.get_param('~username', 'admin')
    password = rospy.get_param('~password', 'admin')
    tag_frame = rospy.get_param('~tag_frame', 'tag_3')

    #  Use the parsed arguments to create the DeviceConnection
    args = argparse.Namespace(ip=ip, username=username, password=password)
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        example_move_to_home_position(base)
        
        rospy.init_node('arm_control_node', anonymous=True)
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer)
        broadcaster = tf2_ros.TransformBroadcaster()

        rate = rospy.Rate(10.0)  # 10 Hz

        max_speeds = [0.05, 0.05, 0.05, 15,15,15]
         # PID coefficients for position and orientation
        Kp_pos = np.array([0.5, 0.5, 0.5])
        Ki_pos = np.array([0.01, 0.01, 0.01])
        Kd_pos = np.array([0.1, 0.1, 0.1])
        max_integral_pos = np.array([0.1, 0.1, 0.1])
        max_output_pos = np.array(max_speeds[:3])
        
        Kp_orient = np.array([0.8, 0.8, 0.8])
        Ki_orient = np.array([0.1, 0.1, 0.1])
        Kd_orient = np.array([0.3, 0.3, 0.3])
        max_integral_orient = np.array([8, 8 ,8])
        max_output_orient = np.array(max_speeds[3:])
        
        pid_pos = PIDController(Kp_pos, Ki_pos, Kd_pos, max_integral_pos, max_output_pos)
        pid_orient = PIDController(Kp_orient, Ki_orient, Kd_orient, max_integral_orient, max_output_orient)
        pid_controllers = [pid_pos, pid_orient]
        

        dt = 0.1  # seconds
        pre_velocity = np.zeros(6)

        while not rospy.is_shutdown():
            try:
                #获取二维码在end_effector_link坐标系下的位姿
                transform = tf_buffer.lookup_transform("end_effector_link", tag_frame, rospy.Time(0))
                # rospy.loginfo(f"Received transform: {transform}")

                # Define the target_pose in the tag_frame
                _target_pose = geometry_msgs.msg.PoseStamped()
                _target_pose.header.frame_id = tag_frame
                _target_pose.pose.position.x = 0
                _target_pose.pose.position.y = 0
                _target_pose.pose.position.z = 0.35
                # (roll,pitch,yaw) ->(x,y,z,w)
                #这里的旋转适用于x向左，y向上，z向前的坐标系
                quat = tf_conversions.transformations.quaternion_from_euler(-3.14, 0, -3.14)
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
                t.header.frame_id = "end_effector_link"
                t.child_frame_id = "transformed_target_pose"
                t.transform.translation.x = target_pose.pose.position.x
                t.transform.translation.y = target_pose.pose.position.y
                t.transform.translation.z = target_pose.pose.position.z
                t.transform.rotation = target_pose.pose.orientation

                # Broadcast the transform
                broadcaster.sendTransform(t)

                '''
                # Implement your control logic here based on the transform
                # For example, you can extract position and orientation information
                # and send control commands to your robot arm
                '''
                # feedback = base_cyclic.RefreshFeedback()
                current_pose = [0,0,0,0,0,0]
                # current_pose[0] = feedback.base.tool_pose_x
                # current_pose[1] = feedback.base.tool_pose_y
                # current_pose[2] = feedback.base.tool_pose_z
                # current_pose[3] = feedback.base.tool_pose_theta_x
                # current_pose[4] = feedback.base.tool_pose_theta_y
                # current_pose[5] = feedback.base.tool_pose_theta_z


                euler = tf_conversions.transformations.euler_from_quaternion([
                    target_pose.pose.orientation.x,
                    target_pose.pose.orientation.y,
                    target_pose.pose.orientation.z,
                    target_pose.pose.orientation.w
                ])

                target_pose_list=[
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z,
                    (euler[0] * 180 / 3.14159),
                    (euler[1] * 180 / 3.14159),
                    (euler[2] * 180 / 3.14159)
                ]

                if  target_pose_list[0] < 0.01 or  \
                    target_pose_list[0] > -0.01 or \
                    target_pose_list[1] < 0.01 or  \
                    target_pose_list[1] < -0.15 or \
                    target_pose_list[3] > 30 or  \
                    target_pose_list[3] < -30 or \
                    target_pose_list[4] > 30 or  \
                    target_pose_list[4] < -30 :
                    velocity = pre_velocity

                if  target_pose_list[0] > 0.2 or  \
                    target_pose_list[0] < -0.2 or \
                    target_pose_list[1] > 0.15 or  \
                    target_pose_list[1] < -0.15 or \
                    target_pose_list[3] > 30 or  \
                    target_pose_list[3] < -30 or \
                    target_pose_list[4] > 30 or  \
                    target_pose_list[4] < -30 :
                    velocity = pre_velocity
                else :
                    velocity = control_robot(target_pose_list, current_pose, max_speeds, dt, pid_controllers)
                    

                print("Velocity command:", velocity)

                twist_command(base, velocity)

                pre_velocity = velocity
                

            except Exception as e:
                rospy.logerr(f"Failed to lookup transform: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        arm_control_node()
    except rospy.ROSInterruptException:
        pass

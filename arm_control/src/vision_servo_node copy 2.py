#!/usr/bin/env python3

import rospy
import sys
import os
import argparse
import time
import threading
import numpy as np
import socket
import signal
import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import geometry_msgs.msg
import tf_conversions
from collections import deque
TIMEOUT_DURATION = 20.0 # seconds

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

    rate = rospy.Rate(10.0)  # 50 Hz

    # Set up the socket server
    server_address = ('192.168.1.102', 11234)  # Update with the correct IP and port
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse the address immediately
    sock.bind(server_address)
    sock.listen(1)

    def close_socket(signum, frame):
        sock.close()
        rospy.signal_shutdown("Socket closed and node shutdown")
        print("Socket closed and node shutdown")

    # Register the signal handler for SIGINT
    signal.signal(signal.SIGINT, close_socket)

    print(f"Listening on {server_address[0]}:{server_address[1]}...")
    # init_pose=geometry_msgs.msg.PoseStamped()
    q_poses=deque(maxlen=8)
    while not rospy.is_shutdown():
        try:
            print("Waiting for a connection...")
            sock.settimeout(None)  # Remove any timeout to wait indefinitely
            connection, client_address = sock.accept()
            print(f"Connection from {client_address}")

            message = f"{0},{0},{0}," \
                        f"{0},{0}," \
                        f"{0},{0}\n"  
            test = 0
            
            # Your existing code here to process the connection
            while not rospy.is_shutdown():
                
                is_result_valid = True
                
                try:
                    connection.sendall(message.encode('utf-8'))
                except socket.error as e:
                    rospy.logerr(f"Socket error during connection: {e}")
                    break
                
                try:
                    #获取二维码在end_effector_link坐标系下的位姿
                    transform = tf_buffer.lookup_transform("wrist", tag_frame, rospy.Time(0))
                    # rospy.loginfo(f"Received transform: {transform}")
                except Exception as e:
                    is_result_valid = False
                    rospy.logerr(f"Failed to lookup transform: {e}")
                    continue
                
                # Define the target_pose in the tag_frame
                _target_pose = geometry_msgs.msg.PoseStamped()
                _target_pose.header.frame_id = tag_frame
                _target_pose.pose.position.x = 0
                _target_pose.pose.position.y = 0
                _target_pose.pose.position.z = 0
                # (roll,pitch,yaw) ->(x,y,z,w)
                #这里的旋转角适用于目标姿态为x向前，y向左，z向上的坐标系
                quat = tf_conversions.transformations.quaternion_from_euler(-1.57, 1.57, 0)
                _target_pose.pose.orientation.x = quat[0]
                _target_pose.pose.orientation.y = quat[1]
                _target_pose.pose.orientation.z = quat[2]
                _target_pose.pose.orientation.w = quat[3]

                # Transform the target_pose to the end_effector_link frame
                target_pose = transform_pose(_target_pose, transform)
                rospy.loginfo(f"Received Pose:")
                
                #发布到tf树便于调试
                # Create a TransformStamped message
                #t = geometry_msgs.msg.TransformStamped()
                #t.header.stamp = rospy.Time.now()
                #t.header.frame_id = "wrist"
                #t.child_frame_id = "target_pose"
                #t.transform.translation.x = target_pose.pose.position.x
                #t.transform.translation.y = target_pose.pose.position.y
                #t.transform.translation.z = target_pose.pose.position.z
               	#t.transform.rotation = target_pose.pose.orientation

                # Broadcast the transform
                #broadcaster.sendTransform(t)

                # Send the target_pose via socket
                try:
                    if transform in q_poses and len(q_poses)==8 and q_poses[0]==q_poses[7]:
                        rospy.loginfo(f"\tOut of the view, Not tag found in view")
                        message = f"0, 0, 0, 0, 0, 0, -2\n" 
                    else:
                        target_pose.pose.position.x += -0.01
                        target_pose.pose.position.y += 0.068
                        target_pose.pose.position.z += 0.0
                        # t2w refers to 'Tag to wrist'
                        pose_t2w = target_pose.pose
                        pos_t2w = pose_t2w.position
                        ori_t2w = pose_t2w.orientation
                        rospy.loginfo(f"\tPos(xyz ): {np.round(pos_t2w.x,3)}, {np.round(pos_t2w.y,3)}, {np.round(pos_t2w.z,3)}")
                        rospy.loginfo(f"\tOri(xyzw): {np.round(ori_t2w.x,2)}, {np.round(ori_t2w.y,2)}, {np.round(ori_t2w.z,2)}, {np.round(ori_t2w.w,2)}")
                        message = f"{pos_t2w.x},{pos_t2w.y},{pos_t2w.z}, {ori_t2w.x},{ori_t2w.y}, {ori_t2w.z},{ori_t2w.w}\n" 
                except Exception as e:
                    print(e)
                    continue
                    
                try:
                    connection.sendall(message.encode('utf-8'))
                    q_poses.append(transform)
                except socket.error as e:
                    rospy.logerr(f"Socket error during connection: {e}")
                    break
                rate.sleep()

        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            continue
        finally:
                connection.close()

    sock.close()
if __name__ == '__main__':
    try:
        vision_servo_node()
    except rospy.ROSInterruptException:
        pass

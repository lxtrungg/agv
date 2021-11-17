#!/usr/bin/env python3

import rospy, tf
import roslib
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
from geometry_msgs.msg import TransformStamped, Vector3, PoseWithCovarianceStamped
import PyKDL
from numpy import sin, cos, deg2rad
from math import atan2

vel = {'v': 0.0 , 'w': 0.0}
pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
allow_initialpose_pub = False
imu_done = False
vel_done = False
imu_yaw = 0

def publish_odometry(position, rotation):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position = Point(*position)
    odom.pose.pose.orientation = Quaternion(*rotation)
    odom.twist.twist.linear  = Vector3(vel['v'], 0, 0)
    odom.twist.twist.angular = Vector3(0, 0, vel['w'])

    odom_pub.publish(odom)

def transform_odometry(position, rotation):  
    trans = TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'odom'
    trans.child_frame_id = 'base_footprint'
    trans.transform.translation = Vector3(*position)
    trans.transform.rotation = Quaternion(*rotation)

    br.sendTransformMessage(trans)

def publisher_initialpose(position, rotation):
    initialpose = PoseWithCovarianceStamped()
    initialpose.header.stamp = rospy.Time.now()
    initialpose.header.frame_id = 'map'
    initialpose.pose.pose.position = Point(*position)
    initialpose.pose.pose.orientation = Quaternion(*rotation)

    init_pose_pub.publish(initialpose)


def subscriber_vel_callback(vel_data):
    global vel_done, vel
    vel_done = True
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

def subscriber_imu_callback(imu_data):
    global imu_done, imu_yaw
    imu_done = True
    imu_yaw = imu_data.z

def dead_reckoning(pose):
    global previous_time
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time
    # pose['x']   += vel['v']*dt*cos(pose['yaw'] + 0.5*vel['w']*dt)
    # pose['y']   += vel['v']*dt*sin(pose['yaw'] + 0.5*vel['w']*dt)
    # pose['yaw']  += vel['w']*dt
    # pose['yaw'] = atan2(sin(pose['yaw']), cos(pose['yaw']))
    pose['x'] += vel['v']*dt*cos(imu_yaw)
    pose['y'] += vel['v']*dt*sin(imu_yaw)
    pose['yaw'] = imu_yaw
    
    return pose

def main():
    global odom_pub, init_pose_pub, br, previous_time, pose
    rospy.init_node('node_odom')
    odom_pub = rospy.Publisher('/agv/odom', Odometry, queue_size=10)
    start_imu_pub = rospy.Publisher('/agv/start_imu', Bool, queue_size=10)
    init_pose_pub = rospy.Publisher('/agv/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.Subscriber('/agv/vel_pub', Twist, subscriber_vel_callback)
    rospy.Subscriber('/agv/imu/rpy', Vector3, subscriber_imu_callback)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(50)
    previous_time = rospy.Time.now()
    while not rospy.is_shutdown():
        if not imu_done:
            start_imu_pub.publish(Bool(True))
        if vel_done and imu_done:
            pose = dead_reckoning(pose)
            rospy.loginfo(pose)
            position = (pose['x'], pose['y'], 0)
            rotation = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
            publish_odometry(position, rotation)
            transform_odometry(position, rotation)
        rate.sleep()

if __name__ == '__main__':
    main()



#!/usr/bin/env python3

import PyKDL
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from localizer_dwm1001.msg import Tag
import tf
import numpy as np
from numpy import sin, cos, deg2rad
from filter.fusionEKF import FusionEKF

pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
vel = {'v': 0.0, 'w': 0.0}
allow_initialpose_pub = False
imu_done = uwb_done = False
first_scan = False
ready_to_pub = False

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

    odom_broadcaster.sendTransformMessage(trans)

def publisher_initialpose(position, rotation):
    initialpose = PoseWithCovarianceStamped()
    initialpose.header.stamp = rospy.Time.now()
    initialpose.header.frame_id = 'map'
    initialpose.pose.pose.position = Point(*position)
    initialpose.pose.pose.orientation = Quaternion(*rotation)

    initialpose_pub.publish(initialpose)

def subscriber_uwb_callback(uwb_data):
    global previous_time, first_scan, ready_to_pub, pose, uwb_done
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time

    if not first_scan:
        if not imu_done:
            return
        first_scan = True
        ekf_uwb.x = np.array([[uwb_data.x], [uwb_data.y], [imu_data_yaw]])
        return

    if dim_z == 1:
        z = np.array([imu_data_yaw])
    elif dim_z == 2:
        z = np.array([[uwb_data.x], [uwb_data.y]])
    else:
        z = np.array([[uwb_data.x], [uwb_data.y], [imu_data_yaw]])
        
    u = np.array([[vel['v']], [vel['w']]])
    x_posterior = ekf_uwb.predict_update(z, u, dt)
    pose['x']   = x_posterior[0,0]
    pose['y']   = x_posterior[1,0]
    pose['yaw'] = x_posterior[2,0]

    uwb_done = True

def subscriber_vel_callback(vel_data):
    global vel
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

def subscriber_imu_callback(imu_data):
    global imu_data_yaw, imu_done
    imu_data_yaw = imu_data.z
    imu_done = True

def subcriber_amcl_callback(amcl_data):
    global allow_initialpose_pub
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    if abs(x - pose['x']) > 0.03 or abs(y - pose['y']) > 0.03 or abs(yaw - pose['yaw']) > np.deg2rad(1):
        allow_initialpose_pub = True

def main():
    rospy.init_node('node_ekf_localization')
    global odom_pub, initialpose_pub, odom_broadcaster
    global allow_initialpose_pub, vel

    odom_pub = rospy.Publisher('/agv/odom_ekf', Odometry, queue_size=10)
    initialpose_pub = rospy.Publisher('/agv/initialpose', PoseWithCovarianceStamped, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber('/agv/vel_pub', Twist, subscriber_vel_callback)
    rospy.Subscriber('/agv/imu_fake', Vector3, subscriber_imu_callback)
    rospy.Subscriber('/agv/uwb/tag', Tag, subscriber_uwb_callback)
    rospy.Subscriber('/agv/amcl_pose', PoseWithCovarianceStamped, subcriber_amcl_callback)

    global previous_time
    previous_time = rospy.Time.now()

    rospy.loginfo('Start node ekf_localization')
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        if imu_done and uwb_done:
            position = (pose['x'], pose['y'], pose['yaw'])
            rotation = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
            publish_odometry(position, rotation)
            # transform_odometry(position, rotation)

            if allow_initialpose_pub:
                allow_initialpose_pub = False
                # publisher_initialpose(position, rotation)
            rospy.loginfo(pose)
        rate.sleep()
    
if __name__ =='__main__':
    std_x = 0.15
    std_y = 0.15
    std_theta = deg2rad(1)
    std_v = 0.03
    std_w = 0.02

    dim_x = 3
    dim_z = 3
    #UWB+IMU
    ekf_uwb = FusionEKF(dim_x=dim_x, dim_z=3)
    ekf_uwb.P = np.diag([.01, .01, .001])
    ekf_uwb.Q = np.diag([std_v, std_w])**2
    if dim_z == 1:
        ekf_uwb.R = np.diag([std_theta])**2
    elif dim_z == 2:
        ekf_uwb.R = np.diag([std_x, std_y])**2
    else:
        ekf_uwb.R = np.diag([std_x, std_y, std_theta])**2
    
    #IMU
    ekf_imu = FusionEKF(dim_x=dim_x, dim_z=1)
    ekf_imu.P = np.diag([.01, .01, .001])
    ekf_imu.R = np.diag([std_theta])**2
    ekf_imu.Q = np.diag([std_v, std_w])**2
    main()
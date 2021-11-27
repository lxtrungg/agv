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
uwb_done = False
ekf_done = False
first_scan = False
sub_data = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

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
    global sub_data, uwb_done
    sub_data['x'] = uwb_data.x
    sub_data['y'] = uwb_data.y
    uwb_done = True

def subscriber_vel_callback(vel_data):
    global vel
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

def subscriber_imu_callback(imu_data):
    global sub_data
    sub_data['yaw'] = imu_data.z
    # compute_ekf_localization()

def subcriber_amcl_callback(amcl_data):
    global allow_initialpose_pub
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    if abs(x - pose['x']) > 0.03 or abs(y - pose['y']) > 0.03 or abs(yaw - pose['yaw']) > np.deg2rad(1):
        allow_initialpose_pub = True

def compute_ekf_localization():
    global previous_time, first_scan, pose, uwb_done, ekf_done
    global x_posterior, P_posterior
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time

    # if not first_scan:
    #     if not imu_done:
    #         return
    #     first_scan = True
    #     ekf_all.x = np.array([[uwb_data.x], [uwb_data.y], [imu_data_yaw]])
    #     return

    u = np.array([[vel['v']], [vel['w']]])
    if not uwb_done:
        z = np.array([sub_data['yaw']])
        x_posterior, P_posterior = ekf_imu.predict_update(x_posterior, P_posterior, z, u, dt)
    else:
        z = np.array([[sub_data['x']], [sub_data['y']], [sub_data['yaw']]])
        x_posterior, P_posterior = ekf_all.predict_update(x_posterior, P_posterior, z, u, dt)
        uwb_done = False

    pose['x']   = x_posterior[0,0]
    pose['y']   = x_posterior[1,0]
    pose['yaw'] = x_posterior[2,0]
    ekf_done = True

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
        compute_ekf_localization()
        if ekf_done:
            position = (pose['x'], pose['y'], pose['yaw'])
            rotation = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
            publish_odometry(position, rotation)
            # transform_odometry(position, rotation)
            # if allow_initialpose_pub:
            #     allow_initialpose_pub = False
                # publisher_initialpose(position, rotation)
            rospy.loginfo(pose)
        rate.sleep()
    
if __name__ =='__main__':
    dim_x = 3
    std_x = 0.15
    std_y = 0.15
    std_theta = deg2rad(1)
    std_v = 0.03
    std_w = 0.02
    P = np.diag([.01, .01, .001])**2
    Q = np.diag([std_v, std_w])**2

    #IMU
    ekf_imu = FusionEKF(dim_x=dim_x, dim_z=1)
    ekf_imu.P = P
    ekf_imu.Q = Q
    ekf_imu.R = np.diag([std_theta])**2

    #UWB
    ekf_uwb = FusionEKF(dim_x=dim_x, dim_z=2)
    ekf_uwb.P = P
    ekf_uwb.Q = Q
    ekf_uwb.R = np.diag([std_x, std_y])**2
    
    #UWB+IMU
    ekf_all = FusionEKF(dim_x=dim_x, dim_z=3)
    ekf_all.P = P
    ekf_all.Q = Q
    ekf_all.R = np.diag([std_x, std_y, std_theta])**2

    x_posterior = np.zeros((3, 1))
    P_posterior = P

    main()
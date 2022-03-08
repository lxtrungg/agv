#!/usr/bin/env python3

import PyKDL
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from localizer_dwm1001.msg import Tag
import tf
import numpy as np
from numpy import sin, cos, deg2rad
from filter.fusionUKF import FusionUKF
from filter.sigma_points import SigmaPoints

# pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
# pose = {'x': 0.0, 'y': 0.5, 'yaw': 0.0}
pose = {'x': 0.0, 'y': 0.8, 'yaw': 0.0}
# pose = {'x': 0.0, 'y': 1.2, 'yaw': 0.0}
# pose = {'x': 0.9, 'y': 0.85, 'yaw': 0.0}
vel =  {'v': 0.0, 'w': 0.0}
allow_initialpose_pub = False
uwb_done = False
vel_done = False
imu_done = False
ukf_done = False
first_scan = False
sub_data = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
process_time = 0

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
    global vel, vel_done
    vel_done = True
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

def subscriber_imu_callback(imu_data):
    global sub_data, imu_done
    imu_done = True
    sub_data['yaw'] = imu_data.z
    # compute_ukf_localization()

def subcriber_amcl_callback(amcl_data):
    global allow_initialpose_pub
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    if abs(x - pose['x']) > 0.03 or abs(y - pose['y']) > 0.03 or abs(yaw - pose['yaw']) > np.deg2rad(1):
        allow_initialpose_pub = True

def compute_ukf_localization():
    global previous_time, first_scan, pose, uwb_done, ukf_done, process_time
    global x_posterior, P_posterior
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time

    # if not first_scan:
    #     if not imu_done:
    #         return
    #     first_scan = True
    #     ukf_all.x = np.array([uwb_data.x, uwb_data.y, imu_data_yaw])
    #     return

    u = np.array([vel['v'], vel['w']])
    if not uwb_done:
        z = np.array([sub_data['yaw']])
        ukf_imu.compute_process_sigmas(x_posterior, P_posterior, u, dt)
        x_posterior, P_posterior = ukf_imu.predict_update(z)
    else:
        z = np.array([sub_data['x'], sub_data['y'], sub_data['yaw']])
        ukf_all.compute_process_sigmas(x_posterior, P_posterior, u, dt)
        x_posterior, P_posterior = ukf_all.predict_update(z)
        uwb_done = False

    pose['x']   = x_posterior[0]
    pose['y']   = x_posterior[1]
    pose['yaw'] = x_posterior[2]
    ukf_done = True

    process_time = (rospy.Time.now() - current_time).to_sec()*1000.0

def main():
    global odom_pub, initialpose_pub, odom_broadcaster
    global allow_initialpose_pub, vel

    rospy.init_node('node_ukf_localization')
    odom_pub = rospy.Publisher('/agv/odom_ukf', Odometry, queue_size=10)
    initialpose_pub = rospy.Publisher('/agv/initialpose', PoseWithCovarianceStamped, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber('/agv/vel_pub', Twist, subscriber_vel_callback)
    rospy.Subscriber('/agv/imu_fake', Vector3, subscriber_imu_callback)
    rospy.Subscriber('/agv/uwb/tag', Tag, subscriber_uwb_callback)
    rospy.Subscriber('/agv/amcl_pose', PoseWithCovarianceStamped, subcriber_amcl_callback)

    global previous_time
    previous_time = rospy.Time.now()

    rospy.loginfo('Start node ukf_localization')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if vel_done and imu_done:
            compute_ukf_localization()
        if ukf_done:
            position = (pose['x'], pose['y'], pose['yaw'])
            rotation = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
            publish_odometry(position, rotation)
            # transform_odometry(position, rotation)
            # if allow_initialpose_pub:
            #     allow_initialpose_pub = False
                # publisher_initialpose(position, rotation)
            print(pose)
            print(process_time)
        rate.sleep()
    
if __name__ =='__main__':
    dim_x = 3
    std_x = 0.28
    std_y = 0.28
    std_theta = deg2rad(1)
    P = np.diag([.1, .1, .01])
    Q = np.diag([.01, .01, .02])**2

    points = SigmaPoints(dim_x=dim_x, alpha=.00001, beta=2, kappa=0)
    #IMU
    ukf_imu = FusionUKF(dim_x=dim_x, dim_z=1, points=points)
    ukf_imu.Q = Q
    ukf_imu.R = np.diag([std_theta])**2

    #UWB
    ukf_uwb = FusionUKF(dim_x=dim_x, dim_z=2, points=points)
    ukf_uwb.Q = Q
    ukf_uwb.R = np.diag([std_x, std_y])**2
    
    #UWB+IMU
    ukf_all = FusionUKF(dim_x=dim_x, dim_z=3, points=points)
    ukf_all.Q = Q
    ukf_all.R = np.diag([std_x, std_y, std_theta])**2

    x_posterior = np.array([pose['x'], pose['y'], pose['yaw']])
    P_posterior = P

    main()
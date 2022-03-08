#!/usr/bin/env python3

import rospy
import PyKDL
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TransformStamped
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from localizer_dwm1001.msg import Tag
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, atan2

shape = np.zeros((3, 1))
x_DR = shape
x_UWB = shape
x_AMCL = shape
x_True = shape
x_EKF = shape
x_EnKF = shape
x_UKF = shape
x_TEST = shape
th_IMU = []
move_done = False
imu_done = False
vel_done = False
# pose = {'x': 0.0, 'y': 0.5, 'yaw': 0.0}
pose = {'x': 0.0, 'y': 0.8, 'yaw': 0.0}
# pose = {'x': 0.0, 'y': 1.2, 'yaw': 0.0}
# pose = {'x': 0.9, 'y': 0.85, 'yaw': 0.0}

# move_x = [0.9, 4.8, 4.8, 0.9, 0.9]
# move_y = [0.85, 0.85, 2.95, 2.95, 0.85]
move_x = [0.0, 4.0]
# move_y = [0.5, 0.5]
move_y = [0.8, 0.8]
# move_y = [1.2, 1.2]
vel = {'v': 0.0 , 'w': 0.0}
process_time = 0
def subscriber_uwb_callback(uwb_data):
    global x_UWB
    pose = np.array([[uwb_data.x], [uwb_data.y], [uwb_data.z]])
    x_UWB = np.hstack((x_UWB, pose))

def subscriber_amcl_callback(amcl_data):
    global x_AMCL
    x = amcl_data.pose.pose.position.x
    y = amcl_data.pose.pose.position.y
    rot = amcl_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_AMCL = np.hstack((x_AMCL, pose))

def subscriber_odom_callback(odom_data):
    global x_True
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    rot = odom_data.pose.pose.orientation
    yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_True = np.hstack((x_True, pose))
  
def subscriber_odom_ekf_callback(odom_data):
    global x_EKF
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    yaw = odom_data.pose.pose.position.z
    # rot = odom_data.pose.pose.orientation
    # yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_EKF = np.hstack((x_EKF, pose))

def subscriber_odom_enkf_callback(odom_data):
    global x_EnKF
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    yaw = odom_data.pose.pose.position.z
    # rot = odom_data.pose.pose.orientation
    # yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_EnKF = np.hstack((x_EnKF, pose))

def subscriber_odom_ukf_callback(odom_data):
    global x_UKF
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    yaw = odom_data.pose.pose.position.z
    # rot = odom_data.pose.pose.orientation
    # yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_UKF = np.hstack((x_UKF, pose))

def subscriber_odom_test_callback(odom_data):
    global x_TEST
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    yaw = odom_data.pose.pose.position.z
    # rot = odom_data.pose.pose.orientation
    # yaw = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
    pose = np.array([[x], [y], [yaw]])
    x_TEST = np.hstack((x_TEST, pose))

def subscriber_imu_callback(imu_data):
    global th_IMU, imu_done
    imu_done = True
    th_IMU = np.hstack((th_IMU, imu_data.z))

def subscriber_vel_callback(vel_data):
    global vel, vel_done
    vel_done = True
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

def subscriber_move_callback(done_data):
    global move_done
    move_done = done_data.data

def dead_reckoning():
    global previous_time, x_DR, pose, process_time
    current_time = rospy.Time.now()
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time
    pose['x']   += vel['v']*dt*cos(pose['yaw'] + 0.5*vel['w']*dt)
    pose['y']   += vel['v']*dt*sin(pose['yaw'] + 0.5*vel['w']*dt)
    pose['yaw']  += vel['w']*dt
    # pose['yaw'] = atan2(sin(pose['yaw']), cos(pose['yaw']))
    arr = np.array([[pose['x'], pose['y'], pose['yaw']]]).T
    x_DR = np.hstack((x_DR, arr))
    process_time = (rospy.Time.now() - current_time).to_sec()*1000.0

def calculate_rmse(a, b, isUWB=False, isIMU=False):
    if isIMU is True:
        N = min(a.shape[0], b[2, 1:].shape[0])
        diff = a[:N] - b[2, 1:N+1]
        for i, s in enumerate(diff):
            diff[i] = atan2(sin(s), cos(s))
        yaw_rmse = np.sqrt(np.square(diff).mean())
        return yaw_rmse
    N = min(a.shape[1], b.shape[1])
    x_rmse = np.sqrt(np.square(a[0, 1:N] - b[0, 1:N]).mean())
    y_rmse = np.sqrt(np.square(a[1, 1:N] - b[1, 1:N]).mean())

    if isUWB is True:
        return x_rmse, y_rmse
        
    diff = a[2, 1:N] - b[2, 1:N]
    for i, s in enumerate(diff):
        diff[i] = atan2(sin(s), cos(s))
    yaw_rmse = np.sqrt(np.square(diff).mean())
    return x_rmse, y_rmse, yaw_rmse

def show_plot():
    plt.figure(figsize=(12, 7))
    # plt.subplot(121)
    plt.title('X-Y Graph')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.axis('equal')
    plt.grid('-')
    linewidth = 1.8
    marksize = 20
    plt.plot(move_x, move_y, 'o-.k', label='xy_True')

    plt.scatter(x_UWB[0, 1:].flatten(), x_UWB[1, 1:].flatten(), marker='*', s=marksize, c='g', label='xy_UWB')

    plt.scatter(x_DR[0, 1:].flatten(), x_DR[1, 1:].flatten(), marker=',', s=marksize, c='y', label='xy_DR')

    plt.scatter(x_True[0, 1:].flatten(), x_True[1, 1:].flatten(), marker='.', s=marksize, c='r', label='xy_DR+IMU')

    # plt.scatter(x_EKF[0, 1:].flatten(), x_EKF[1, 1:].flatten(), marker='+', s=marksize, c='c', label='xy_EKF')

    plt.scatter(x_TEST[0, 1:].flatten(), x_TEST[1, 1:].flatten(), marker='o', s=marksize, c='m', label='xy_EKF')

    plt.scatter(x_EnKF[0, 1:].flatten(), x_EnKF[1, 1:].flatten(), marker='v', s=marksize, c='k', label = 'xy_EnKF')

    plt.scatter(x_UKF[0, 1:].flatten(), x_UKF[1, 1:].flatten(), marker='x', s=marksize, c='b', label = 'xy_UKF')

    plt.legend(loc='upper right')
    
    # plt.subplot(122)
    # plt.title('YAW Graph')
    # plt.xlabel('Sample (n)')
    # plt.ylabel('Yaw (degree)')
    # plt.ylim(-180, 180)
    # plt.grid(True)
    
    # plt.plot(np.rad2deg(th_IMU), '-g', linewidth = linewidth, label='yaw_IMU')
    # plt.plot(np.rad2deg(x_DR[2, 1:].flatten()), '-y', linewidth = linewidth, label='yaw_DR')
    # plt.plot(np.rad2deg(x_TEST[2, 1:].flatten()), '-c', linewidth = linewidth, label='yaw_HIEN')
    # plt.plot(np.rad2deg(x_EKF[2, 1:].flatten()), '-m', linewidth = linewidth, label='yaw_EKF')
    # plt.plot(np.rad2deg(x_EnKF[2, 1:].flatten()), '-k', linewidth = linewidth, label='yaw_EnKF')
    # plt.plot(np.rad2deg(x_UKF[2, 1:].flatten()), '-b', linewidth = linewidth, label='yaw_UKF')
    # plt.legend(loc='upper right')
    plt.show()

def main():
    rospy.init_node('node_debug')
    rospy.Subscriber('/agv/uwb/tag', Tag, subscriber_uwb_callback)
    rospy.Subscriber('/agv/amcl_pose', PoseWithCovarianceStamped, subscriber_amcl_callback)
    rospy.Subscriber('/agv/odom', Odometry, subscriber_odom_callback)
    # rospy.Subscriber('/agv/odom_ekf', Odometry, subscriber_odom_ekf_callback)
    rospy.Subscriber('/agv/odom_enkf', Odometry, subscriber_odom_enkf_callback)
    rospy.Subscriber('/agv/odom_ukf', Odometry, subscriber_odom_ukf_callback)
    rospy.Subscriber('/agv/odom_test', Odometry, subscriber_odom_test_callback)
    # rospy.Subscriber('/my_robot/imu/rpy/filtered', Vector3Stamped, subscriber_imu_callback)
    rospy.Subscriber('/agv/imu_fake', Vector3, subscriber_imu_callback)
    rospy.Subscriber('/agv/vel_pub', Twist, subscriber_vel_callback)
    rospy.Subscriber('/agv/move_done', Bool, subscriber_move_callback)

    rospy.loginfo('Start node plot_debug')
    rate = rospy.Rate(10)
    global x_DR, x_UWB, x_AMCL, x_True
    global move_done
    global vel, previous_time
    previous_time = rospy.Time.now()
    while not rospy.is_shutdown():
        if vel_done and imu_done:
            dead_reckoning()
            print(pose)
            print(process_time)

        if move_done:
            move_done = False
            # print('RMSE IMU :' + str(calculate_rmse(th_IMU, x_True, isIMU=True)))
            # print('RMSE UWB :' + str(calculate_rmse(x_UWB, x_True, isUWB=True)))
            # print('RMSE DR  :' + str(calculate_rmse(x_DR, x_True)))
            # print('RMSE EKF :' + str(calculate_rmse(x_EKF, x_True)))
            # print('RMSE EKF2 :' + str(calculate_rmse(x_TEST, x_True)))
            print('RMSE EnKF:' + str(calculate_rmse(x_EnKF, x_True)))
            # print('RMSE UKF :' + str(calculate_rmse(x_UKF, x_True)))
            rospy.loginfo('Show plot')
            show_plot()
            break
        
        rate.sleep()

if __name__ =='__main__':
    main()
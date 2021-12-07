#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import matplotlib.pyplot as plt
import random
import statistics as stat
data = []
plot_done = False

def subscriber_imu_callback(imu_data):
    data.append(imu_data.z)

def show_plot(event):
    mean = np.mean(data)
    std = np.std(data)
    rospy.loginfo('mean: '+ str(mean))
    rospy.loginfo('std: '+ str(std))
    plt.figure(figsize=(10,8))
    plt.subplot(121)
    plt.title('Sampling IMU data')
    plt.xlabel('Sample (n)')
    plt.ylabel('Value (degree)')
    plt.grid('-')
    _linewidth = 1.2
    
    plt.plot(np.rad2deg(data),'-r', linewidth=_linewidth, label='IMU_yaw')
    plt.legend(loc='upper right')
    rospy.loginfo('Show plot')
    plt.show()
    plot_done = True
    
def main():
    rospy.init_node('node_sampling_imu')
    rospy.Subscriber('/agv/imu/rpy', Vector3, subscriber_imu_callback)
    rospy.loginfo('Sampling IMU data...')
    previous_time = rospy.Time.now()
    rospy.Timer(rospy.Duration(3600), show_plot, oneshot=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if plot_done:
            break
        rate.sleep()

if __name__=='__main__':
    main()
#!/usr/bin/env python3

import rospy
from localizer_dwm1001.msg import Tag
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
import random
data = {'x': [], 'y': []}
plot_done = False
def subscriber_uwb_callback(uwb_data):
    # data['x'].append(uwb_data.x)
    # data['y'].append(uwb_data.y)
    x = random.uniform(0.0, 0.1)
    y = random.uniform(0.0, 0.1)
    data['x'].append(x)
    data['y'].append(y)

def show_plot(event):
    x_mean = np.mean(data['x'])
    x_std = np.std(data['x'])
    y_mean = np.mean(data['y'])
    y_std = np.std(data['y'])
    rospy.loginfo('x_mean: ' + str(x_mean))
    rospy.loginfo('x_std: ' + str(x_std))
    rospy.loginfo('y_mean: ' + str(y_mean))
    rospy.loginfo('y_std: ' + str(y_std))
    plt.figure(figsize=(18,8))
    plt.subplot(121)
    plt.title('Sampling UWB data')
    plt.xlabel('Sample (n)')
    plt.ylabel('Value (m)')
    plt.grid('-')
    _linewidth = 1.2
    
    plt.plot(data['x'], '-r', linewidth=_linewidth, label='UWB_X')
    plt.plot(data['y'], '-g', linewidth=_linewidth, label='UWB_Y')
    plt.legend(loc='upper right')
    
    plt.subplot(122)
    plt.title('Position UWB data')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.grid('-')
    
    plt.scatter(0, 0, label='XY_ROBOT')
    plt.scatter(data['x'], data['y'], label='XY_UWB')
    plt.scatter(x_mean, y_mean, label='XY_MEAN')
    # n, bins, patches = plt.hist(data['x'], bins=20, density=False, histtype='bar', rwidth=0.7, color='blue', alpha=0.4)
    plt.legend(loc='upper right')
    rospy.loginfo('Show plot')
    plt.show()
    plot_done = True
    
def main():
    rospy.init_node('node_sampling_uwb')
    rospy.Subscriber('/agv/uwb/tag', Tag, subscriber_uwb_callback)
    rospy.loginfo('Sampling UWB data...')
    previous_time = rospy.Time.now()
    rospy.Timer(rospy.Duration(5), show_plot, oneshot=False)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if plot_done:
            break
        rate.sleep()
    # rospy.spin()

if __name__=='__main__':
    main()
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

from urllib.request import urlopen, Request
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# sp_left = []
# vel_left = []
# sp_right = []
# vel_right = []
speed = {'left': 0.0, 'right': 0.0}
setpoint = {'left': 0.0, 'right': 0.0}
# class LivePlotter(object):
#     def __init__(self):
#         self.fig = plt.figure()
#         self.fig.tight_layout()
#         self.ax1 = self.fig.add_subplot(1, 2, 1)
#         self.ax2 = self.fig.add_subplot(1, 2, 2)
#         self.time_interval = 20
#         self.data =  { 'sp_left': [], 'vel_left': [], 'sp_right': [], 'vel_right': []}

#     def add_point_data(self, sp_left, vel_left, sp_right, vel_right):
#         self.data['sp_left'].append(sp_left)
#         self.data['vel_left'].append(vel_left)
#         self.data['sp_right'].append(sp_right)
#         self.data['vel_right'].append(vel_right)
#         print(self.data['vel_right'])

#     def animation(self, i):
#         pass
#         # for data in self.data['sp_left']:
            
#         self.ax1.clear()
#         self.ax1.plot(self.data['vel_right'])
#         # self.ax1.plot(self.data['vel_right'])

#     def show(self):
#         self.ani = FuncAnimation(self.fig, self.animation, interval=self.time_interval)
#         plt.show()

def subcriber_vol_callback(vol_data):
    global speed
    speed['left'] = vol_data.linear.x
    speed['right'] = vol_data.angular.z
    # plot.add_point_data(0, vol_data.linear.x, 0, vol_data.angular.z*10)
    
def subscriber_sp_callback(sp_data):
    setpoint['left'] = sp_data.linear.x
    setpoint['right'] = sp_data.angular.z

def get_value():
    request = Request('http://192.168.11.12:8080/edIhvxcUqeDcPQ2cgpqtqWo_nsR6fGo6/get/V1')
    response_body = urlopen(request).read()
    print(response_body)


def set_value():
    request = Request('http://192.168.11.12:8080/g3tN7ELDKhTyjIadNRAfkfCDHovSOgay/update/V0?value=' + str(speed['left']))
    response_body = urlopen(request).read()
    request = Request('http://192.168.11.12:8080/g3tN7ELDKhTyjIadNRAfkfCDHovSOgay/update/V1?value=' + str(speed['right']))
    response_body = urlopen(request).read()
    request = Request('http://192.168.11.12:8080/edIhvxcUqeDcPQ2cgpqtqWo_nsR6fGo6/update/V0?value=' + str(setpoint['left']))
    response_body = urlopen(request).read()
    request = Request('http://192.168.11.12:8080/edIhvxcUqeDcPQ2cgpqtqWo_nsR6fGo6/update/V1?value=' + str(setpoint['right']))
    response_body = urlopen(request).read()

def main():
    rospy.init_node('node_pid_graph')
    rospy.Subscriber('/agv/vol_pub', Twist, subcriber_vol_callback)
    rospy.Subscriber('/agv/sp_pub', Twist, subscriber_sp_callback)
    # plot.show()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        set_value()
        # get_value()
        rate.sleep()

if __name__ == '__main__':
    # plot = LivePlotter()
    main()
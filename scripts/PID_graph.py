#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

from urllib.request import urlopen, Request
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

speed = {'left': 0.0, 'right': 0.0}
setpoint = {'left': 0.0, 'right': 0.0}
class LivePlotter(object):
    def __init__(self):
        self.fig = plt.figure(figsize=(15,12))
        self.fig.tight_layout()
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        self.ax2 = self.fig.add_subplot(2, 1, 2)
        self.time_interval = 20
        self.cnt = 0
        self.time = 0
        self.data =  { 'time': [], 'sp_left': [], 'vel_left': [], 'sp_right': [], 'vel_right': []}

    def add_point_data(self, sp_left, vel_left, sp_right, vel_right):
        self.cnt += 1
        self.time = self.cnt*self.time_interval/1000.0
        self.data['time'].append(self.time)
        self.data['sp_left'].append(sp_left)
        self.data['vel_left'].append(vel_left)
        self.data['sp_right'].append(sp_right)
        self.data['vel_right'].append(vel_right)

    def animation(self, i):
        self.ax1.clear()
        self.ax1.plot(self.data['time'], self.data['sp_left'])
        self.ax1.plot(self.data['time'], self.data['vel_left'])
        self.ax1.plot(self.data['time'], self.data['vel_right'])
        self.ax1.set_xlabel('time (s)')
        self.ax1.set_ylabel('speed (m/s)')
        self.ax1.grid(visible=True, which='major', axis='both')
        self.ax2.clear()
        self.ax2.plot(self.data['time'], self.data['sp_right'])
        self.ax2.plot(self.data['time'], self.data['vel_right'])
        self.ax2.set_xlabel('time (s)')
        self.ax2.set_ylabel('speed (m/s)')
        self.ax2.grid(visible=True, which='major', axis='both')
        if self.time <= 10:
            self.ax1.set_xlim(0, 10.0)
            self.ax2.set_xlim(0, 10.0)
        else:
            self.ax1.set_xlim(self.time - 7.0, self.time + 3.0)
            self.ax2.set_xlim(self.time - 7.0, self.time + 3.0)

    def show(self):
        self.ani = FuncAnimation(self.fig, self.animation, interval=self.time_interval)
        plt.show()

def subcriber_vol_callback(vol_data):
    global speed
    speed['left'] = vol_data.linear.x
    speed['right'] = vol_data.angular.z
    plot.add_point_data(setpoint['left'], vol_data.linear.x, setpoint['right'], vol_data.angular.z)
    
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
    plot.show()
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
        # set_value()
        # get_value()
        # rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    plot = LivePlotter()
    main()
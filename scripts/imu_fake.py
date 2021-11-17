#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from math import atan2, sin, cos
pre_imu_raw = 0
imu_fake = 0

def subcriber_imu_callback(imu_data):
    global pre_imu_raw
    imu_raw = imu_data.z
    delta = imu_raw - pre_imu_raw
    delta = atan2(sin(delta), cos(delta))
    pre_imu_raw = imu_raw
    publish_imu_fake(delta)
    
def publish_imu_fake(delta):
    global imu_fake
    imu_fake += delta
    data = Vector3()
    data.z = imu_fake
    print(imu_fake)
    imu_pub.publish(data)

def main():
    global imu_pub
    rospy.init_node('node_imu_fake')
    imu_pub = rospy.Publisher('/agv/imu_fake', Vector3, queue_size=10)
    rospy.Subscriber('/agv/imu/rpy', Vector3, subcriber_imu_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
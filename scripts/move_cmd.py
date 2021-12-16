#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from math import pi

yaw = 0
imu_done = False
move_done = False
def publish_cmd_vel(v, w):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    cmd_vel_pub.publish(msg)

def move_rec():
    global move_done
    delta = (rospy.Time.now() - previous_time).to_sec()
    v = 0.0
    w = 0.0
    v_ref = 0.4
    w_ref = 1.0

    if delta <= 5.0:
        v = v_ref
        w = 0.0
    elif delta >= 7.0 and delta <= 10.0:
        if yaw <= pi/2 - 0.57:
            v = 0.0
            w = w_ref
        else:
            v = 0.0
            w = 0.0

    elif delta >= 12.0 and delta <= 16.5:
        v = v_ref
        w = 0.0
    elif delta >= 18.5 and delta <= 21.5:
        if abs(yaw) <= pi - 0.6:
            v = 0.0
            w = w_ref
        else:
            v = 0.0
            w = 0.0

    elif delta >= 23.5 and delta <= 28.4:
        v = v_ref
        w = 0.0
    elif delta >= 30.5 and delta <= 33.5:
        if abs(yaw) >= pi/2 + 0.6:
            v = 0.0
            w = w_ref
        else:
            v = 0.0
            w = 0.0

    elif delta >= 35.5 and delta <= 39.85:
        v = v_ref
        w = 0.0
    elif delta >= 42.0 and delta <= 45.0:
        if abs(yaw) >= 0 + 0.6:
            v = 0.0
            w = w_ref
        else:
            v = 0.0
            w = 0.0
            move_done = True

    elif delta >= 46.0:
        pass
    else:
        v = 0.0
        w = 0.0

    publish_cmd_vel(v, w)

def callback(msg_data):
    global yaw, imu_done
    yaw = msg_data.z
    imu_done = True
    print(yaw)

def main():
    global previous_time, cmd_vel_pub, move_done_pub
    rospy.init_node('node_move_cmd')
    start_imu_pub = rospy.Publisher('/agv/start_imu', Bool, queue_size=10)
    move_done_pub = rospy.Publisher('/agv/move_done', Bool, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/agv/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/agv/imu/rpy', Vector3, callback)
    
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        if not imu_done:
            # start_imu_pub.publish(Bool(True))
            previous_time = rospy.Time.now()
        else:
            move_rec()
        if move_done:
            move_done_pub.publish(Bool(True))
            break
        rate.sleep()

if __name__ == '__main__':
    main()
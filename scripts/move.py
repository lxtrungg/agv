#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from std_msgs.msg import Bool
from math import pi, atan
import PyKDL

vel = {'v': 0.0 , 'w': 0.0}

rec_A_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0}
rec_B_pose = {'x': 1.6, 'y': 0.0, 'yaw': pi/2}
rec_C_pose = {'x': 1.6, 'y': 1.6, 'yaw': pi}
rec_D_pose = {'x': 0.0, 'y': 1.6, 'yaw': -pi/2}
phi = atan(1/7)
num_A_pose = {'x': 3.5, 'y': 3.5, 'yaw': -pi/2}
num_B_pose = {'x': 3.5, 'y': 0.5, 'yaw': -(pi - phi)}
num_C_pose = {'x': -3.5, 'y': -0.5, 'yaw': -pi/2}
num_D_pose = {'x': -3.5, 'y': -3.5, 'yaw': 0}
num_E_pose = {'x': 3.5, 'y': -3.5, 'yaw': pi/2}
num_F_pose = {'x': 3.5, 'y': -0.5, 'yaw': (pi - phi)}
num_G_pose = {'x': -3.5, 'y': 0.5, 'yaw': pi/2}
num_H_pose = {'x': -3.5, 'y': 3.5, 'yaw': 0}

def publish_goal(pose):
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    pos = (pose['x'], pose['y'], 0)
    rot = PyKDL.Rotation.RPY(0, 0, pose['yaw']).GetQuaternion()
    goal.pose.position = Point(*pos)
    goal.pose.orientation = Quaternion(*rot)
    rospy.loginfo('Publish goal pose: ' + str(pose))
    goal_pub.publish(goal)

def get_pose_rec(i):
    switch = {
        0: rec_B_pose,
        1: rec_C_pose,
        2: rec_D_pose,
        3: rec_A_pose
    }
    return switch.get(i)

def get_pose_num(i):
    switch = {
        0: num_B_pose,
        1: num_C_pose,
        2: num_D_pose,
        3: num_E_pose,
        4: num_F_pose,
        5: num_G_pose,
        6: num_H_pose,
        7: num_A_pose
    }
    return switch.get(i)

def subcriber_vel_callback(vel_data):
    vel['v'] = vel_data.linear.x
    vel['w'] = vel_data.angular.z

def move_circle():
    cmd_vel = Twist()
    vel_v = 0.492
    vel_w = 0.189
    cmd_vel.linear.x = vel_v
    cmd_vel.angular.z = -vel_w
    
    time_start = rospy.Time.now()
    rate = rospy.Rate(20)
    time_done = 5*pi/vel_w
    while not rospy.is_shutdown():
        time_duaration = (rospy.Time.now() - time_start).to_sec()
        if time_duaration <= 55:
            vel_pub.publish(cmd_vel)
        else:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            vel_pub.publish(cmd_vel)
            return 10
        print(time_duaration)
        rate.sleep()

def move_rec(step):
    if step <= 3:
        publish_goal(get_pose_rec(step))
        step += 1
    else:
        return 10
    return step

def move_num(step):
    if step <= 7:
        publish_goal(get_pose_num(step))
        step += 1
    else:
        return 10
    return step

def main():
    global goal_pub, vel_pub
    rospy.init_node('node_move')
    goal_pub = rospy.Publisher('/agv/move_base_simple/goal', PoseStamped, queue_size=10)
    debug_pub = rospy.Publisher('/agv/move_done', Bool, queue_size=10)
    vel_pub = rospy.Publisher('/agv/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/agv/cmd_vel', Twist, subcriber_vel_callback)
    cnt = 0
    step = 0
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if vel['v'] == 0 and vel['w'] == 0:
            if cnt <= 100:
                cnt += 1
            else:
                cnt = 0
                step = move_rec(step)
                # step = move_num(step)
                # step = move_circle()
                
                if step == 10:
                    debug_pub.publish(Bool(True))
        else:
            cnt = 0

        rate.sleep()

if __name__ =='__main__':
    main()

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    global previous_time
    if msg.linear.x == 0.0 and msg.angular.z == 0.0:
        previous_time = rospy.Time.now()
    elif msg.linear.x >= 0.001 or msg.angular.z >= 0.001:
        print((rospy.Time.now() - previous_time).to_sec())
        
def main():
    rospy.init_node('node_time_test')
    rospy.Subscriber('/agv/vel_pub', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
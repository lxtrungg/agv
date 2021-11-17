#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Vector3, Point
from nav_msgs.msg import Odometry
from localizer_dwm1001.msg import Tag

pose = Point()
uwb_data_raw = Vector3()
uwb_data_tf  = Vector3()

def subscriber_odom_callback(data):
    pose = data.pose.pose.position

def subscriber_uwb_callback(data):
    uwb_data_raw.x = data.x
    uwb_data_raw.y = data.y
    uwb_data_raw.z = data.z

def main():
    rospy.init_node('node_uwb_transform')
    listener = tf.TransformListener()
    uwb_pub = rospy.Publisher('/agv/uwb_pub', Vector3, queue_size=10)
    rospy.Subscriber('/agv/uwb/tag', Tag, subscriber_uwb_callback)
    rospy.Subscriber('/agv/odom', Odometry, subscriber_odom_callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time.now())
            # pose.x += trans[0]
            # pose.y += trans[1]
            # pose.z += trans[2]
            print(trans)
        except:
            continue
        rate.sleep()


if __name__ == '__main__':
    main()
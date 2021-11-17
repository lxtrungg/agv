#! /usr/bin/env python3

from geometry_msgs.msg import Twist
import threading
import rospy
import sys, select, termios, tty

LINEAR_STEP = 0.1
ANGULAR_STEP = 0.1
current_time = 0
previous_time = 0
class PublishThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.publisher = rospy.Publisher('/agv/cmd_vel', Twist, queue_size=10)
        self.x = 0.0
        self.th = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.start()

    def wait_for_subcribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 0 and i == 20:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
                rospy.sleep(0.5)
                i+=1
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, linear_vel, angular_vel):
        self.condition.acquire()
        self.x = linear_vel
        self.th = angular_vel
        self.condition.release()

    def run(self):
        pass
        # twist = Twist()
        # while not self.done:
        #     self.condition.acquire()
        #     twist.linear.x = self.x
        #     twist.linear.y = 0.0
        #     twist.linear.z = 0.0
        #     twist.angular.x = 0.0
        #     twist.angular.y = 0.0
        #     twist.angular.z = self.th
        #     self.condition.release()
            # self.publisher.publish(twist)

    def publish(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th
        self.publisher.publish(twist)

    def stop(self):
        self.done = True
        self.update(0.0, 0.0)
        self.join()


def getKey():
    try:
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('agv_teleop')
    publish_thread = PublishThread()

    linear_speed = 0.3
    angular_speed = 1
    linear_vel = 0.0
    angular_vel  = 0.0
    global current_time, previous_time
    previous_time = rospy.Time.now()
    try:
        publish_thread.wait_for_subcribers()
        publish_thread.update(linear_vel, angular_vel)
        while True:
            current_time = rospy.Time.now()
            key = getKey()
            if key =='w':
                linear_vel = linear_speed
                angular_vel = 0.0
                print('FORWARD   ', end='\r')
            elif key =='s':
                linear_vel = -linear_speed
                angular_vel = 0.0
                print('BACKWARD  ', end='\r')
            elif key =='d':
                angular_vel = -angular_speed
                linear_vel = 0.0
                print('RIGHTWARD ', end='\r')
            elif key =='a':
                angular_vel = angular_speed
                linear_vel = 0.0
                print('LEFTWARD  ', end='\r')
            elif key =='z':
                linear_speed += LINEAR_STEP
                print('INCREASE',  end='\r')
            elif key =='x':
                linear_speed -= LINEAR_STEP
                print('DECREASE',  end='\r')
            elif key == 'q':
                linear_vel = 0.0
                angular_vel = 0.0
                print('STOP     ',  end='\r')
            else:
                if (key == '\x03'):
                    break
            
            publish_thread.update(linear_vel, angular_vel)
            if ((current_time - previous_time).to_sec() >= 0.1):
                publish_thread.publish()
                previous_time = current_time
    finally:
        publish_thread.stop()

if __name__ == '__main__':
    main()

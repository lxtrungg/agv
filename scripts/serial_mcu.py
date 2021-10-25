#! /usr/bin/env python3
import struct
import serial
import rospy
from geometry_msgs.msg import Twist

v = 0
w = 0
cnt = 0

class ReadnWriteFromSerial(object): 
    def __init__(self, serial):
        self.ser = serial
        self.value = []
 
    def read_one_struct(self):
            myByte = self.ser.read(1)
            if myByte == b'S':  # Look for Start indicator
                data = self.ser.read_until(b'E') # Z indicates Stop
                data = data[1:len(data)-1]
                if len(data) == 8:
                    self.value = struct.unpack('<ff', data)
                    return True
                else:
                    return False
            else:
                return False
    
    def write_one_struct(self, v, w):
        myByte = struct.pack('<ff', v, w)
        self.ser.write(myByte)

def subscriber_cmd_callback(cmd_data):
    global v, w
    v = cmd_data.linear.x
    w = cmd_data.angular.z
    serClass.write_one_struct(v, w)

def publish_vel(value):
    vel = Twist()
    vel.linear.x = value[0]
    vel.angular.z = value[1]
    vel_pub.publish(vel)
    
def main():
    global serClass, vel_pub
    rospy.init_node('agv_serial')
    vel_pub = rospy.Publisher('/agv/vel_pub', Twist, queue_size=10)
    rospy.Subscriber('/agv/cmd_vel', Twist, subscriber_cmd_callback)
    ser = serial.Serial(
        port     = str(rospy.get_param('~serial_port_name', '/dev/ttyUSB0')),
        baudrate = int(rospy.get_param('~serial_baud_rate', 115200)),
        parity   = serial.PARITY_ODD,
        stopbits = serial.STOPBITS_TWO,
        bytesize = serial.SEVENBITS)

    rate = rospy.Rate(50)
    serClass = ReadnWriteFromSerial(ser)
    while not rospy.is_shutdown():
        if (serClass.read_one_struct()):
            publish_vel(serClass.value)
        rospy.sleep()

if __name__ == "__main__":
    main()
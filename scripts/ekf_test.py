#!/usr/bin/env python3

import rospy
import roslib
import tf

import PyKDL
import numpy as np
import math
from math import sin, cos, pi
from numpy import deg2rad
from std_msgs.msg import String, Int8, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3Stamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from localizer_dwm1001.msg import Tag

# Covariance for EKF simulation
Q = np.diag([
    0.01,            # variance of location on x-axis
    0.01,            # variance of location on y-axis
    0.02  # variance of yaw angle - 0.017453292519943295
]) ** 2             # predict state covariance

R_UWB              = np.diag([0.28, 0.28]) ** 2  # Observation x,y position covariance
# Q = np.diag([0.03, 0.02])**2
#--------------------------------------

class EKF_Publisher:
    def __init__(self):
        rospy.init_node('Node_EKF_Publisher', anonymous=True)
        #Pub
        self.odom_pub               = rospy.Publisher('/agv/odom_test', Odometry, queue_size= 50)
        self.odom_broadcaster_tf    = tf.TransformBroadcaster()
        #Sub
        rospy.Subscriber('/agv/vel_pub',      Twist,      self.vel_callback)
        rospy.Subscriber('/agv/uwb/tag',       Tag,      self.uwb_sub_callback)
        rospy.Subscriber('/agv/imu_fake',   Vector3,    self.rpyFilter_callback)

        self.frame_id = rospy.get_param('~frame_id','/odom')    
        self.child_frame_id = rospy.get_param('~child_frame_id','/base_footprint')


        self.vel = {'v':0.0 , 'w':0.0} #v robot (m/s)  && w robot (rad/s)

        #for predict EKF
        self.time_ekf_update = rospy.Time.now()
        self.PEst = np.eye(3)*10 
        self.xEst = np.zeros((3, 1))

        self.pose    = {'x': 0.0, 'y': 0.0, 'th': 0.0}
        self.posexDR = {'x': 0.0, 'y': 0.0, 'th': 0.0}
        
        #pre-updte
        self.xEst[0,0] = self.pose['x']
        self.xEst[1,0] = self.pose['y']
        self.xEst[2,0] = self.pose['th']

        self.ready = False
        self.yaw_imu = 0.0

        self.yaw_bias_timevw = rospy.Time.now()

        #R,Q
        # self.R_IMU  = 0.1
        self.R_IMU  = deg2rad(1)

        self.uwb_msg = 0
        self.yaw = 0
        self.uwb_done = False
        
    def update(self):
        if self.ready == True:
            self.pub_odometry(self.pose)
    
    def pub_odometry(self,pose):
        current_time = rospy.Time.now()

        # Construct odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time            # self.last_time  == current_time
        odom_msg.header.frame_id = self.frame_id        #"odom"
        odom_msg.child_frame_id = self.child_frame_id   #"base_footprint"
        # set the position
        odom_msg.pose.pose.position = Point(pose['x'], pose['y'], pose['th'])
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
        # set the velocity
        odom_msg.twist.twist = Twist(Vector3(self.vel['v'], 0, 0), Vector3(0, 0, self.vel['w']))
         # publish the message
        self.odom_pub.publish(odom_msg)

    def vel_callback(self,vel_msg):
        self.vel['v'] = vel_msg.linear.x    # v robot (m/s)
        self.vel['w'] = vel_msg.angular.z   # w robot (rad/s)
    
    def rpyFilter_callback(self, yaw_msg):
        # yaw = yaw_msg.z
        # current_time = rospy.Time.now()
        # dt = (current_time - self.yaw_bias_timevw).to_sec()
        # self.yaw_bias_timevw = rospy.Time.now()

        # self.R_IMU  = 0.1 
        # u = np.array([[self.vel['v']], [self.vel['w']]])
        # z = np.array([yaw])
        # self.xEst, self.PEst = self.ekf_estimation_imu(self.xEst, self.PEst, z, u)
        
        # # Update to Odometry
        # self.pose['x'] = self.xEst[0,0]
        # self.pose['y'] = self.xEst[1,0]
        # self.pose['th'] = self.xEst[2,0]
        #
        self.ready = True   

        self.yaw = yaw_msg.z

    def uwb_sub_callback(self, uwb_msg):
        
        # u = np.array([[self.vel['v']], [self.vel['w']]])
        # z = np.array([[uwb_msg.x],[uwb_msg.y]])

        # self.xEst, self.PEst = self.ekf_estimation_uwb(self.xEst, self.PEst, z, u)
        # # Update to Odometry
        # self.pose['x'] = self.xEst[0,0]
        # self.pose['y'] = self.xEst[1,0]
        # self.pose['th'] = self.xEst[2,0]
        #
        self.ready = True
        self.uwb_msg = uwb_msg
        self.uwb_done = True
    
    def compute_ekf_localization(self):
        u = np.array([[self.vel['v']], [self.vel['w']]])
        if not self.uwb_done:
            z = np.array([self.yaw])
            self.xEst, self.PEst = self.ekf_estimation_imu(self.xEst, self.PEst, z, u)
        else:
            z = np.array([[self.uwb_msg.x],[self.uwb_msg.y]])
            self.xEst, self.PEst = self.ekf_estimation_uwb(self.xEst, self.PEst, z, u)
            self.uwb_done = False
            
        self.pose['x'] = self.xEst[0,0]
        self.pose['y'] = self.xEst[1,0]
        self.pose['th'] = self.xEst[2,0]


    def spin(self): #ok
        rospy.loginfo("[ROS][hiennd] Start NodeEKF_Publisher")
        rate = rospy.Rate(50)   
        while not rospy.is_shutdown():
            self.compute_ekf_localization()
            self.update()           
            print(self.pose) 
            rate.sleep()
    #------------------------------------
    #----------------EKF (UWB) (asyn)----------------
    # robot system model --------------
    def robot_model_system(self,x, u,Ts):
        # Ref: (2.20) chapter (2.2.2)
        w = u[1,0]
        theta = x[2,0]
        # F = [3x3]
        F = np.array([[1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])  
        # B = [3x2]
        B = np.array([[Ts*math.cos(theta + 0.5*Ts*w),       0],
                    [Ts*math.sin(theta + 0.5*Ts*w),         0],
                    [0.0,                                   Ts]])
        # = [3x3][3x1] + [3x2][2x1] = [3x1]
        #x = F @ x + B @ u
        x = F.dot(x) + B.dot(u)

        #
        return x
    # Matrix Fk [3x3] -------------------------------
    def jacob_f(self, x, u,Ts):
        v       = u[0, 0]     # v - robot
        w       = u[1, 0]     # w - robot
        theta   = x[2,0]      # yaw
        #jF = 3x3
        jF = np.array([
            [1.0,   0.0,    -Ts * v * math.sin(theta + 0.5*Ts*w)],
            [0.0,   1.0,     Ts * v * math.cos(theta + 0.5*Ts*w)],
            [0.0,   0.0,                                    1.0]])
        return jF

    def jacob_w(self, x, u,Ts):
        v       = u[0, 0]     # v - robot
        w       = u[1, 0]     # w - robot
        theta   = x[2,0]      # yaw
        #jW = 3x2
        jW = np.array([[Ts*math.cos(theta + 0.5*Ts*w), -0.5*v*(Ts**2)*math.sin(theta + 0.5*Ts*w)],
                       [Ts*math.sin(theta + 0.5*Ts*w),  0.5*v*(Ts**2)*math.cos(theta + 0.5*Ts*w)],
                       [                            0,                                        Ts]])
        return jW

    def jacob_h_uwb(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0] ])
        return jH
    def jacob_h_imu(self):
        # Jacobian of Observation Model
        jH = np.array([
            [0, 0, 1]])   
        return jH
    # H(UWB-IMU) [3x3] -----------------------
    def jacob_h_amcl(self):
        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]])
        return jH
    #----------------EKF-------------------------
    def ekf_estimation_uwb(self,xEst, PEst, z, u):
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time
        #rospy.loginfo("[DEBUG][ekf_estimation_uwb]---------------: " + str(Ts)) #PASSED
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + Q
        # jW = self.jacob_w(xPred, u, Ts)
        # PPred = jF.dot(PEst).dot(jF.T) + jW.dot(Q).dot(jW.T)

        #  Update
        jH = self.jacob_h_uwb()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + R_UWB
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        xEst = xPred + K.dot((z - zPred)) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        return xEst, PEst
    #-----------------
    def ekf_estimation_imu(self,xEst, PEst, z, u):
        current_time = rospy.Time.now()
        Ts = (current_time - self.time_ekf_update).to_sec()   #deta_time
        self.time_ekf_update = current_time                   #update time
        #  Predict
        xPred = self.robot_model_system(xEst, u, Ts)
        jF = self.jacob_f(xPred, u, Ts)
        PPred = jF.dot(PEst).dot(jF.T) + Q
        # jW = self.jacob_w(xPred, u, Ts)
        # PPred = jF.dot(PEst).dot(jF.T) + jW.dot(Q).dot(jW.T)

        #  Update    
        jH = self.jacob_h_imu()  #IMU ()
        zPred = jH.dot(xPred)
        S = jH.dot(PPred).dot(jH.T) + self.R_IMU
        K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    
        y = z - zPred
        xEst = xPred + K.dot(y) #
        PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

        return xEst, PEst   
# -------------Main--------------------
def main():
    ekf = EKF_Publisher()
    ekf.spin()

if __name__ == '__main__':
    main()
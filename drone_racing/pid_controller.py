#!/usr/bin/env python

"""
======== IMPORTS ========
"""
from __future__ import print_function

import rospy

import os
import time
from collections import deque
import numpy as np

from nav_msgs.msg import Odometry
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from gates_positions import PositionFinder

##=================================

"""
TO DO:
[] Finish implementing <list> target position
"""

class PID:

    def __init__(self,Kp=1,Ki=1,Kd=0):
        """
        Init PID class
        """

        #=====PID Parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.P = 0
        self.I = 0
        self.D = 0
        self.last_error = 0

        #=====Values
        self.curr = 0
        self.des = 0
        self.current_time = time.time()
        self.last_time = self.current_time
        self.output = 0

    def feedback(self,current, desired, current_time=None):

        self.curr = current
        self.des = desired
        self.current_time = current_time if current_time is not None else time.time()

        # Calculate the PID errors
        error = self.des - self.curr
        delta_t = self.current_time - self.last_time
        delta_e = error - self.last_error

        # Update time and error
        self.last_time = self.current_time
        self.last_error = error

        # Calculate output
        self.P = self.Kp * error
        self.I += error * delta_t
        self.D = delta_e / delta_t
        self.output = self.P + self.Ki * self.I + self.Kd * self.D 

class DroneController:
    def __init__(self,target="gate",target_pos=None):
        """
        Init Control class
        """
        print('=============== Drone controller init ===============')

        #=====Controller
        self.x_control = PID(0.6,0.02,0)
        self.y_control = PID(0.6,0.02,0)
        self.z_control = PID(1.5,0.2,0)
        #self.yaw_control = PID(1,1,0)

        #=====ROS
        self.rate = rospy.Rate(20)
        self.vel_msg = Twist()
        self.enable_motors()
        self.pos_sub = rospy.Subscriber('/ground_truth/state', 
                                        Odometry, 
                                        self.callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', 
                                       Twist, 
                                       queue_size=10)
        #rospy.init_node('pid_controller', anonymous=True)

        #====Trajectory
        self.targets = []

        if target == "gate":
            gates_pos = PositionFinder()
            xt = gates_pos.x[0]
            yt = gates_pos.y[0]
            zt = gates_pos.z[0]

        elif target == "position":
            xt = target_pos[0]
            yt = target_pos[1]
            zt = target_pos[2]

        elif target == "list":
            for pos in target_pos:
                xt = target_pos[pos][0]
                yt = target_pos[pos][1]
                zt = target_pos[pos][2]
                self.targets.append([xt,yt,zt])

        else:
            print("ERROR: Enter valid input")
            print("target must be <gate>, <position> or <list>")

        self.set_target(x=xt,y=yt,z=zt,yaw=None)

    def callback(self, msg):
        """
        Callback function for the Subscriber
        """
        self.x = np.array([msg.pose.pose.position.x])
        self.y = np.array([msg.pose.pose.position.y])
        self.z = np.array([msg.pose.pose.position.z])
        self.fly()

    def set_target(self,x,y,z,yaw=None):
        """
        Get target position for drone
        """
        self.x_des = x
        self.y_des = y
        self.z_des = z
        #self.yaw_des = 0

    def controller_feedback(self):
        """
        Compute the output for each variable
        """
        try:
            self.x_control.feedback(self.x, self.x_des)
            self.y_control.feedback(self.y, self.y_des)
            self.z_control.feedback(self.z, self.z_des)
            #self.yaw_control.feedback(self.yaw, self.yaw_des)     
        except AttributeError:
            pass

    def enable_motors(self):
        """
        Function for enabling motors
        """
        rospy.wait_for_service('/enable_motors')
        enable_motor = rospy.ServiceProxy('/enable_motors', EnableMotors)
        try:
            enable_motor(True)
            print('===============  Motors enabled =======================')

        except rospy.ServiceException as exc:
            print('Service did not process request: ', str(exc))

    def write_vel(self):
        """
        Write velocity message
        """
        self.vel_msg.linear.x = self.x_control.output
        self.vel_msg.linear.y = self.y_control.output
        self.vel_msg.linear.z = self.z_control.output
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        #self.vel_msg.angular.z = self.yaw_control.output

    def send_vel(self):
        """
        Function to publish desired velocities
        """
        self.write_vel()
        self.vel_pub.publish(self.vel_msg)

    def fly(self):
        """
        Execute the loop to fly
        """
        self.controller_feedback()
        self.send_vel()
        #print("X = ",self.x, "Y = ",self.y,"Z = ",self.z)


if __name__ == '__main__':
    main()


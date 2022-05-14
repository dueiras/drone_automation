#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy #enables Python programmers with ROS
import numpy as np #enables matrix and array
import os # manage file
import time #time function
from collections import deque
import sys

import cv2 #allows to consider images like pixels matrix
import csv #write csv file


from cv_bridge import CvBridge, CvBridgeError # converts ros images into opencv images
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

"""
TO DO:
[] Make script end after max number of images is reached
"""

class Gate:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class DataSaver:

    def __init__(self,folder_name="data",max_imgs=50,file_name="data.csv",save_rate=20):
        """
        Init class
        """
        print('=============== Init DataSaver ================')

        self.folder_name = folder_name + "/"
        self.file_name = file_name
        self.img_deque = deque()
        self.num_images = 0

        #=====ROS
        self.max_imgs = max_imgs
        self.images = self.max_imgs*[0]
        self.xwrite = np.zeros((self.max_imgs, 1))
        self.ywrite = np.zeros((self.max_imgs, 1))
        self.zwrite = np.zeros((self.max_imgs, 1))
        self.vel_msg = Twist()
        self.bridge = CvBridge()

        self.img_sub = rospy.Subscriber('front_cam/camera/image', Image, self.callback_img)

        self._gateListDict = {

            'gate_a': Gate('adr2017_Small_Gate', 'link_8')

            }

        self.rate = rospy.Rate(save_rate) #maintain a particular rate for a loop

        self.gate_height = 0,7
        self.x_gates = np.array([])
        self.y_gates = np.array([])
        self.z_gates = np.array([])
        self.gate_names = []
        self.count_gates = 0

        self.odometry_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.callback_pos)

        self.start()

    def callback_img(self, data):
        """
        Callback function for the Subscriber
        """
        # Get image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Scale image
        self.img_scaled = cv2.resize(cv_image, (640,360))
        self.img_name = "images/img" + str(self.num_images) + ".jpg"
        cv2.imwrite(self.folder_name+self.img_name, self.img_scaled)
        self.images[self.num_images] = self.img_name
        self.num_images += 1
        if(self.num_images == self.max_imgs):
            self.save_img()
            sys.exit()
            rospy.on_shutdown()

    def start(self):

        print ('Starting Position Finder...')
        self.get_gates_pos()
        self.rate.sleep()

    def callback_pos(self, msg):
        self.x_drone = np.array([msg.pose.pose.position.x])
        self.y_drone = np.array([msg.pose.pose.position.y])
        self.z_drone = np.array([msg.pose.pose.position.z])

        """
        print("Drone: ")
        print("X Position: " + str(self.x_drone))
        print("Y Position: " + str(self.y_drone))
        print("Z Position: " + str(self.z_drone) + '\n')
        """

        if self.num_images <= self.max_imgs:
            self._compute_vector()

    def _compute_vector(self):

        self.xwrite[self.num_images] = self.x_drone[0] - self.x_gates[0]
        self.ywrite[self.num_images] = self.y_drone[0] - self.y_gates[0]
        self.zwrite[self.num_images] = self.z_drone[0] - self.z_gates[0]

    def get_gates_pos(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for gate in self._gateListDict.itervalues():

                gateName = str(gate._name)
                linkName = str(gate._relative_entity_name)

                world_coordinates = model_coordinates(gateName,'world')
                link_coordinates = model_coordinates(gateName,linkName)

                print ('\n')
                print ('Status.success = ', world_coordinates.success)

                xpos = world_coordinates.pose.position.x + link_coordinates.pose.position.x
                ypos = world_coordinates.pose.position.y + link_coordinates.pose.position.y
                zpos = world_coordinates.pose.position.z + link_coordinates.pose.position.z

                print("Gate " + str(gateName))
                print("X Position: " + str(xpos))
                print("Y Position: " + str(ypos))
                print("Z Position: " + str(zpos) + '\n')

                self.gate_names.append(str(gate._name))
                self.x_gates = np.append(self.x_gates, xpos)
                self.y_gates = np.append(self.y_gates, ypos)
                self.z_gates = np.append(self.z_gates, zpos)
                self.count_gates += 1

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    def save_img(self):

        path = self.folder_name + self.file_name
        header = ["Path","X","Y","Z"]
    	with open(path, 'w') as f:
    	    # create the csv writer
       	    writer = csv.writer(f)
            writer.writerow(header)
            for x in range(self.max_imgs):
    	       row = [self.images[x], str(self.xwrite[x][0]), str(self.ywrite[x][0]), str(self.zwrite[x][0])]
    	       writer.writerow(row)
        print("========= ALL IMAGES SAVED =========")            


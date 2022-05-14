#!/usr/bin/env python2

from image_saver import DataSaver
from pid_controller import DroneController
import rospy
import cv2

def main():
    rospy.init_node('dataset_builder', anonymous=True)
    folder_name = "data"
    max_imgs = 50
    save_rate = 30
    saver = DataSaver(folder_name=folder_name,max_imgs=max_imgs,save_rate=save_rate)
    controller = DroneController(target="gate")

    try:
        rospy.spin()    
    except KeyboardInterrupt:
        print('Shutting down')    
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
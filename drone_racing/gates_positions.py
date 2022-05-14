#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
import numpy as np

"""
            'gate_a': Gate('adr2017_Small_Gate_clone_0', 'link_8'),
            'gate_b': Gate('adr2017_Small_Gate', 'link_8'),
            'platform': Gate('platform', 'link'),
            'gate_c': Gate('adr2017_Small_Gate_clone', 'link_8'),
            'gate_d': Gate('adr2017_Small_Gate_clone_clone', 'link_8')

            }
"""

class Gate:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class PositionFinder:

    def __init__(self):
        self._gateListDict = {
            
            'gate_a': Gate('adr2017_Small_Gate', 'link_8')

            }

        self.rate = rospy.Rate(0.1)

        self.gate_height = 0.7
        self.x = np.array([])
        self.y = np.array([])
        self.z = np.array([])
        self.gate_names = []
        self.count_gates = 0
        self.start()

    def start(self):

        print 'Starting Gate Position Finder...'
        self.get_gates_pos()

    def get_gates_pos(self):
        try:

            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for gate in self._gateListDict.itervalues():

                gateName = str(gate._name)
                linkName = str(gate._relative_entity_name)

                world_coordinates = model_coordinates(gateName,'world')
                link_coordinates = model_coordinates(gateName,linkName)

                print '\n'
                print 'Status.success = ', world_coordinates.success

                xpos = world_coordinates.pose.position.x + link_coordinates.pose.position.x
                ypos = world_coordinates.pose.position.y + link_coordinates.pose.position.y
                zpos = world_coordinates.pose.position.z + abs(link_coordinates.pose.position.z) + self.gate_height

                print("Gate " + str(gateName))
                print("X Position: " + str(xpos))
                print("Y Position: " + str(ypos))
                print("Z Position: " + str(zpos) + '\n')
                #print("X Quaternion: " + str(world_coordinates.pose.orientation.x))
                #print("Y Quaternion: " + str(world_coordinates.pose.orientation.y))
                #print("Z Quaternion: " + str(world_coordinates.pose.orientation.z))

                self.gate_names.append(str(gate._name))
                self.x = np.append(self.x, xpos)
                self.y = np.append(self.y, ypos)
                self.z = np.append(self.z, zpos)
                self.count_gates += 1

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e)) 


if __name__ == '__main__':
    main()


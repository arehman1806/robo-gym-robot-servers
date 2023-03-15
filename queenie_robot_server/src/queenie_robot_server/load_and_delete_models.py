#!/usr/bin/env python

import rospy
import os
import rospkg
import sys

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from std_srvs.srv import Empty
import random
from geometry_msgs.msg import Pose
import PyKDL
import numpy as np

class QueenieGraspObjectSetManager(object):
    def __init__(self):

        # Connect to the services for spawning and deleting models
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.rospack = rospkg.RosPack()
        self.models = {}
        for i in range(12):
            self.models["object_" + str(i)] = self.read_sdf("object_" + str(i))
        
        self.loaded_model_name = None

    def read_sdf(self, model_name):
        sdf_path = self.get_path(model_name)
        with open(sdf_path, 'r') as f:
            sdf = f.read()
        return sdf

    def load_model(self, model_id, x=0, y=0, yaw=0):
        model_id = int(model_id)
        if self.loaded_model_name is not None:
            self.delete_model(self.loaded_model_name)
            self.loaded_model_name = None
            rospy.sleep(5)
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = 0.5
        orientation = PyKDL.Rotation.RPY(0,0, yaw)
        initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w = orientation.GetQuaternion()
        # Spawn the model in Gazebo
        try:
            self.spawn_model("object_" + str(model_id), self.models["object_" + str(model_id)], '/', initial_pose, "world")
            self.loaded_model_name = "object_" + str(model_id)
        except:
            rospy.sleep(10)

        rospy.sleep(5)
        

    def delete_model(self, model_name):
        # Delete the model from Gazebo
        self.delete_model(model_name)
    
    def get_path(self, model_name):
        return os.path.join(self.rospack.get_path('queenie'), "src", 'models', 'queenie_grasp_object_set', model_name, 'model.sdf')

# if __name__ == '__main__':
#     rospy.init_node('sdf_loader')

#     # Set the name and path of the SDF model file
#     # model_name = 'my_model'
    
#     # sdf_path = os.path.join(rospack.get_path('queenie'), 'models', 'my_model', 'model.sdf')

#     # Create an instance of the SDFLoader class and load the model
#     loader = SDFLoader()
#     model_name = random.choice([x for x in range(12)])
#     yaw = np.random.uniform(-np.pi/2, np.pi/2)
#     x = np.random.uniform(1, 1.6)
#     y = np.random.uniform(-0.5, 0.5)

#     loader.load_model(model_name, x, y, yaw)
#     exit()

#     while not rospy.is_shutdown():
#         model_name = random.choice([x for x in range(12)])
#         yaw = np.random.uniform(-np.pi/2, np.pi/2)
#         x = np.random.uniform(1, 1.6)
#         y = np.random.uniform(-0.5, 0.5)

#         loader.load_model(model_name, x, y, yaw)

class stateObj:
        def __init__(self) -> None:
                self.state = []

from queenie_robot_server.ros_bridge import RosBridge
import rospy
import math
import numpy as np
import time
import random
rospy.init_node("abc")
def _get_object_pose():

        handle_to_center_translation = np.array([0.55, -1.55, 0])
        yaw_t = np.random.uniform(low=-0.785, high=2.355)
        rot_matrix = np.array([[ np.cos(yaw_t), -np.sin(yaw_t), 0 ],
                                [ np.sin(yaw_t), np.cos(yaw_t) , 0 ],
                                [ 0            , 0             , 1 ]])
        handle_to_center_translation = rot_matrix.dot(handle_to_center_translation)
        
        x_t = np.random.uniform(low=2.5, high=5) + handle_to_center_translation[0]
        max_y = math.tan(1)*x_t /1.5
        y_t = np.random.uniform(low=-max_y, high=max_y) + handle_to_center_translation[1]

        return [x_t,y_t,yaw_t]

x = RosBridge()

# while True:
x.object_manager.load_model(2)
state = [0]*12
state[6:9] = [0.5,0.5, 0.5]
a = stateObj()
a.state = state
x.set_state(a)
# while True:
        # result = x.send_explore_goal()
        # print(result)
# while True:
#     x.set_model_state("large_cuboid", _get_object_pose())
#     # time.sleep(0.5)

rospy.spin()


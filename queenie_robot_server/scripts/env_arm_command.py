#!/usr/bin/env python3
import rospy
import random
# from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray
from queue import Queue

class JointTrajectoryCH:
    def __init__(self):
        rospy.init_node('env_am_commandddd')
        # self.real_robot =  rospy.get_param("~real_robot")
        self.real_robot = False
        # ac_rate = rospy.get_param("~action_cycle_rate")
        ac_rate = 10
        self.rate = rospy.Rate(ac_rate)
        self.x = 0

        # Publisher to JointTrajectory robot controller
        if self.real_robot:
            # self.jt_pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
            # self.jt_pub = rospy.Publisher('/pos_traj_controller/command', JointTrajectory, queue_size=10)
            pass
        else:
            self.jt_pub = rospy.Publisher('/env_arm_command', Float64MultiArray, queue_size=10)

    def joint_trajectory_publisher(self):

        while not rospy.is_shutdown():
            # If a command from the environment is waiting to be executed,
            # publish the command, otherwise preempt trajectory
            msg = Float64MultiArray()
            
            # if self.x == 0:
            #      msg.data = [-0, 10, -10, -10]
            #      self.x = 1
            # else:
            #     msg.data = [0.0, 0, 10, 10]
            #     self.x = 0

            msg = Float64MultiArray()
            msg.data = [0.6, 0, 10, 10]
            self.jt_pub.publish(msg)
            self.rate.sleep()

            msg = Float64MultiArray()
            msg.data = [0.6, 0.1, 10, 10]
            self.jt_pub.publish(msg)
            self.rate.sleep()

            msg = Float64MultiArray()
            msg.data = [0.6, 0.1, -10, -10]
            self.jt_pub.publish(msg)
            self.rate.sleep()

            msg = Float64MultiArray()
            msg.data = [0.6, 0.2, -10, -10]
            self.jt_pub.publish(msg)
            self.rate.sleep()

            msg = Float64MultiArray()
            msg.data = [-0.6, 0.2, -10, -10]
            self.jt_pub.publish(msg)
            self.rate.sleep()
            



if __name__ == '__main__':
    try:
        ch = JointTrajectoryCH()
        ch.joint_trajectory_publisher()
    except rospy.ROSInterruptException:
        pass
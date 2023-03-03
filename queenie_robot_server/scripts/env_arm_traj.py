#!/usr/bin/env python3
import rospy
import random
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header
from queue import Queue

class JointTrajectoryCH:
    def __init__(self):
        rospy.init_node('env_am_commandddd')
        # self.real_robot =  rospy.get_param("~real_robot")
        self.real_robot = False
        # ac_rate = rospy.get_param("~action_cycle_rate")
        ac_rate = 2
        self.rate = rospy.Rate(ac_rate)
        self.x = 0
        
        self.previous_positions = [0.04, -0.02, -10, -10]

        # Publisher to JointTrajectory robot controller
        if self.real_robot:
            # self.jt_pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
            # self.jt_pub = rospy.Publisher('/pos_traj_controller/command', JointTrajectory, queue_size=10)
            pass
        else:
            self.jt_pub = rospy.Publisher('/env_arm_command', JointTrajectory, queue_size=10)
        msg = JointTrajectory()
        msg.joint_names = ["neck", "palm_riser", "palm_left_finger", "palm_right_finger"]
        msg.points=[JointTrajectoryPoint()]
        msg.header =Header()
        msg.points[0].positions = [0.2, 0, 0, 0]
        self.jt_pub.publish(msg)

    def joint_trajectory_publisher(self):

        while not rospy.is_shutdown() and (self.previous_positions[0] > -0.36 and self.previous_positions[1] < 0.15):
            # If a command from the environment is waiting to be executed,
            # publish the command, otherwise preempt trajectory
            # msg = JointTrajectory()
            msg = JointTrajectory()
            msg.joint_names = ["neck", "palm_riser", "palm_left_finger", "palm_right_finger"]
            msg.points=[JointTrajectoryPoint()]
            msg.header =Header()
            msg.points[0].positions = [self.previous_positions[0]-0.04, self.previous_positions[1] + 0.02, 0, 0]
            self.previous_positions[0] -= 0.04
            self.previous_positions[1] += 0.02
            
            # if self.x == 0:
            #     msg.points[0].positions = [0, 0, 0, 0]
            #     self.x = 1
            # else:
            #     msg.points[0].positions = [0, 0, -10, -10]
            #     self.x = 0
            msg.points[0].time_from_start = rospy.Duration.from_sec(0.1)
            self.jt_pub.publish(msg)
            # rospy.sleep(1/10 - 0.01)
            print("hello bhai")
            self.rate.sleep()
        while not rospy.is_shutdown():
            msg = JointTrajectory()
            msg.joint_names = ["neck", "palm_riser", "palm_left_finger", "palm_right_finger"]
            msg.points=[JointTrajectoryPoint()]
            msg.header =Header()
            msg.points[0].positions = [self.previous_positions[0], self.previous_positions[1], 0, 0]
            
            # if self.x == 0:
            #     msg.points[0].positions = [0, 0, 0, 0]
            #     self.x = 1
            # else:
            #     msg.points[0].positions = [0, 0, -10, -10]
            #     self.x = 0
            msg.points[0].time_from_start = rospy.Duration.from_sec(0.1)
            self.jt_pub.publish(msg)
            # rospy.sleep(1/10 - 0.01)
            print("hello bhai")
            self.rate.sleep()

            

            



if __name__ == '__main__':
    try:
        ch = JointTrajectoryCH()
        ch.joint_trajectory_publisher()
    except rospy.ROSInterruptException:
        pass
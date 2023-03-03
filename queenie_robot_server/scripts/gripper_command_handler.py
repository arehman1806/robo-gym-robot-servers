#!/usr/bin/env python3
import rospy
# from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray
from queue import Queue

class JointTrajectoryCH:
    def __init__(self):
        rospy.init_node('gripper_command_handler')
        # self.real_robot =  rospy.get_param("~real_robot")
        self.real_robot = False
        # ac_rate = rospy.get_param("~action_cycle_rate")
        ac_rate = 25
        self.rate = rospy.Rate(ac_rate)

        # Publisher to JointTrajectory robot controller
        if self.real_robot:
            # self.jt_pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
            # self.jt_pub = rospy.Publisher('/pos_traj_controller/command', JointTrajectory, queue_size=10)
            pass
        else:
            self.jt_pub = rospy.Publisher('/queenie/gripper_controller/command', Float64MultiArray, queue_size=10)

        # Subscriber to JointTrajectory Command coming from Environment
        rospy.Subscriber('env_arm_command', Float64MultiArray, self.callback_env_joint_trajectory, queue_size=1)
        self.msg = Float64MultiArray()
        # Queue with maximum size 1
        self.queue = Queue(maxsize=1)
        # Flag used to publish empty JointTrajectory message only once when interrupting execution
        self.stop_flag = False 

    def callback_env_joint_trajectory(self, data):
        try:
            # Add to the Queue the next command to execute
            self.queue.put(data)
        except:
            pass

    def joint_trajectory_publisher(self):

        while not rospy.is_shutdown():
            # If a command from the environment is waiting to be executed,
            # publish the command, otherwise preempt trajectory
            if self.queue.full():
                self.jt_pub.publish(self.queue.get())
                self.stop_flag = False 
            else:
                # If the empty JointTrajectory message has no been published publish it and
                # set the stop_flag to True, else pass
                if not self.stop_flag:
                    # msg = Float64MultiArray()
                    # msg.data = [0,0,0,0]
                    # self.jt_pub.publish(msg)
                    # self.stop_flag = True 
                    # print("publishing a stop message")
                    pass
                else: 
                    pass 
            self.rate.sleep()


if __name__ == '__main__':
    try:
        ch = JointTrajectoryCH()
        ch.joint_trajectory_publisher()
    except rospy.ROSInterruptException:
        pass
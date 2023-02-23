#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int16, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import PyKDL
import tf2_ros
import copy
from tf_conversions import posemath
from threading import Event
import numpy as np
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2

class RosBridge:

    def __init__(self, real_robot=False):

        self.laserlen = 133

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.real_robot = real_robot
        # cmd_vel_command_handler publisher
        self.env_cmd_vel_pub = rospy.Publisher('env_cmd_vel', Twist, queue_size=1)
        # Target RViz Marker publisher
        self.target_pub = rospy.Publisher('target_marker', Marker, queue_size=10)
        # Rviz Path publisher
        self.mir_exec_path = rospy.Publisher('mir_exec_path', Path, queue_size=10)
        # Odometry of the robot subscriber
        if self.real_robot:
            rospy.Subscriber('odom', Odometry, self.callbackOdometry, queue_size=1)
        else:
            rospy.Subscriber('odom', Odometry, self.callbackOdometry, queue_size=1)

        # subscribers

        # rospy.Subscriber('b_scan', LaserScan, self.LaserScanBack_callback)
        # rospy.Subscriber('f_scan', LaserScan, self.LaserScanFront_callback)
        rospy.Subscriber('left_finger_contact', Bool, self.left_finger_contact_callback)
        rospy.Subscriber('right_finger_contact', Bool, self.right_finger_contact_callback)
        rospy.Subscriber('min_distance_to_handle', Float64, self.min_distance_to_handle_callback)
        rospy.Subscriber('angle_to_handle', Float64, self.angle_to_handle_callback)
        rospy.Subscriber('handle_pc_count', Float64, self.handle_pc_count_callback)
        rospy.Subscriber('camera/processed_laser_scan', Float64MultiArray, self.laser_scan_callback)
        rospy.Subscriber('robot_pose', Pose, self.callbackState, queue_size=1)

        self.target = [0.0] * 3
        self.queenie_pose = [0.0] * 3
        self.queenie_twist = [0.0] * 2
        self.visible_handle_points = [0.0]
        self.min_distance_to_handle = [0.0]
        self.angle_to_handle = [0.0]
        self.laser_scan = [0.0] * self.laserlen

        # will extend the state as and when required:
        # self.left_finger_contact = [False]
        # self.right_finger_contact = [False]
        # self.relative_twist = [0.0] * 6
        # self.obstacle_to_manipulate = [0.0] * 3

        # Reference frame for Path
        self.path_frame = 'map'

        # Initialize Path
        self.mir_path = Path()
        self.mir_path.header.stamp = rospy.Time.now()
        self.mir_path.header.frame_id = self.path_frame

        # Flag indicating if it is safe to move backwards
        self.safe_to_move_back = True
        # Flag indicating if it is safe to move forward
        self.safe_to_move_front = True
        self.rate = rospy.Rate(10)  # 30Hz
        self.reset.set()

    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state = []
        target = copy.deepcopy(self.target)
        queenie_pose = copy.deepcopy(self.queenie_pose)
        visible_handle_points = copy.deepcopy(self.visible_handle_points)
        queenie_twist = copy.deepcopy(self.queenie_twist)
        min_distance_to_handle = copy.deepcopy(self.min_distance_to_handle)
        angle_to_handle = copy.deepcopy(self.angle_to_handle)
        laser_scan = copy.deepcopy(self.laser_scan)

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(queenie_pose)
        msg.state.extend(queenie_twist)
        msg.state.extend(visible_handle_points)
        msg.state.extend(min_distance_to_handle)
        msg.state.extend(angle_to_handle)
        msg.state.extend(laser_scan)
        msg.success = 1
        
        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state
        # Clear reset Event
        self.reset.clear()
        # Re-initialize Path
        self.mir_path = Path()
        self.mir_path.header.stamp = rospy.Time.now()
        self.mir_path.header.frame_id = self.path_frame

        # Set target internal value
        self.target = copy.deepcopy(state[0:3])
        # Publish Target Marker
        self.publish_target_marker(self.target)

        if not self.real_robot :
            # Set Gazebo Robot Model state
            self.set_model_state('queenie', copy.deepcopy(state[3:6]))
            # Set Gazebo Target Model state
            self.set_model_state('target', copy.deepcopy(state[0:3]))
            # Set obstacles poses
            self.set_model_state('large_cuboid', copy.deepcopy(state[16:19]))

        # Set reset Event
        self.reset.set()

        if not self.real_robot:
            # Sleep time set manually to allow gazebo to reposition model
            rospy.sleep(0.2)

        return 1

    def publish_env_cmd_vel(self, lin_vel, ang_vel):
        # if (not self.safe_to_move_back) or (not self.safe_to_move_front):
        #     # If it is not safe to move overwrite velocities and stop robot
        #     rospy.sleep(0.07)
        #     return 0.0, 0.0
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        self.env_cmd_vel_pub.publish(msg)
        # Sleep time set manually to achieve approximately 10Hz rate
        rospy.sleep(0.07)
        return lin_vel, ang_vel

    

    def set_model_state(self, model_name, state):
        # Set Gazebo Model State
        rospy.wait_for_service('/gazebo/set_model_state')

        start_state = ModelState()
        start_state.model_name = model_name
        start_state.pose.position.x = state[0]
        start_state.pose.position.y = state[1]
        orientation = PyKDL.Rotation.RPY(0,0, state[2])
        start_state.pose.orientation.x, start_state.pose.orientation.y, start_state.pose.orientation.z, start_state.pose.orientation.w = orientation.GetQuaternion()

        start_state.twist.linear.x = 0.0
        start_state.twist.linear.y = 0.0
        start_state.twist.linear.z = 0.0
        start_state.twist.angular.x = 0.0
        start_state.twist.angular.y = 0.0
        start_state.twist.angular.z = 0.0

        try:
            set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state/', SetModelState)
            set_model_state_client(start_state)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def min_distance_to_handle_callback(self, data):
        self.min_distance_to_handle = [data.data]

    def angle_to_handle_callback(self, data):
        self.angle_to_handle = [data.data]
        

    def handle_pc_count_callback(self, data: Float64):
        self.visible_handle_points = [data.data]
    
    def laser_scan_callback(self, data: Float64MultiArray):
        if self.get_state_event.isSet():
            self.laser_scan = copy.deepcopy(list(data.data))
        else:
            pass

    
    def callbackOdometry(self, data):
        lin_vel = data.twist.twist.linear.x
        ang_vel = data.twist.twist.angular.z

        # Update internal Twist variable
        self.queenie_twist = copy.deepcopy([lin_vel, ang_vel])

    def callbackState(self,data):
        # If state is not being reset proceed otherwise skip callback
        if self.reset.isSet():
            # if self.real_robot:
            #     # Convert Pose from relative to Map to relative to World frame
            #     f_r_in_map = posemath.fromMsg(data)
            #     f_r_in_world = self.world_to_map * f_r_in_map
            #     data = posemath.toMsg(f_r_in_world)

            x = data.position.x
            y = data.position.y

            orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                                 data.orientation.y,
                                                 data.orientation.z,
                                                 data.orientation.w)

            euler_orientation = orientation.GetRPY()
            yaw = euler_orientation[2]

            # # Append Pose to Path
            # stamped_mir_pose = PoseStamped()
            # stamped_mir_pose.pose = data
            # stamped_mir_pose.header.stamp = rospy.Time.now()
            # stamped_mir_pose.header.frame_id = self.path_frame
            # self.mir_path.poses.append(stamped_mir_pose)
            # self.mir_exec_path.publish(self.mir_path)

            # Update internal Pose variable
            self.queenie_pose = copy.deepcopy([x, y, yaw])
        else:
            pass
    
    
    def left_finger_contact_callback(self, data):
        pass
    
    def right_finger_contact_callback(self, data):
        pass

#=========================================================================================================================#

    # NOT IN USE ATM
    def publish_target_marker(self, target_pose):
        # Publish Target RViz Marker
        t_marker = Marker()
        t_marker.type = 2  # =>SPHERE
        t_marker.scale.x = 0.3
        t_marker.scale.y = 0.3
        t_marker.scale.z = 0.3
        t_marker.action = 0
        t_marker.frame_locked = 1
        t_marker.pose.position.x = target_pose[0]
        t_marker.pose.position.y = target_pose[1]
        t_marker.pose.position.z = 0.0
        rpy_orientation = PyKDL.Rotation.RPY(0.0, 0.0, target_pose[2])
        q_orientation = rpy_orientation.GetQuaternion()
        t_marker.pose.orientation.x = q_orientation[0]
        t_marker.pose.orientation.y = q_orientation[1]
        t_marker.pose.orientation.z = q_orientation[2]
        t_marker.pose.orientation.w = q_orientation[3]
        t_marker.id = 0
        t_marker.header.stamp = rospy.Time.now()
        t_marker.header.frame_id = self.path_frame
        t_marker.color.a = 1.0
        t_marker.color.r = 0.0  # red
        t_marker.color.g = 1.0
        t_marker.color.b = 0.0
        self.target_pub.publish(t_marker)

    # NOT IN USE ATM
    def odometry_callback(self, data):
        # Save robot velocities from Odometry internally
        self.robot_twist = data.twist.twist

    # NOT IN USE ATM
    def get_robot_state(self):
        # method to get robot position from real mir
        return self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta, self.robot_twist.linear.x, self.robot_twist.linear.y, self.robot_twist.angular.z
    def LaserScanBack_callback(self, data):
        if self.get_state_event.isSet():
            scan = data.ranges
            scan = scan[10:len(scan)-20]  # remove first 10 and last 20 elements from laser scan because they are 0.0 in scan on real MiR100
            #scan=list(filter(lambda a: a != 0.0, scan))   # remove all 0.0 values that are at beginning and end of scan list
            scan = np.array(scan)
            scan = np.nan_to_num(scan)
            scan = np.clip(scan, data.range_min, data.range_max)
            self.b_scan = copy.deepcopy(scan.tolist())
            self.safe_to_move_back = all(i >= 0.04 for i in scan)
        else:
            pass

    def LaserScanFront_callback(self,data):
        if self.get_state_event.isSet():
            scan = data.ranges
            scan = scan[30:len(scan)-10] # remove first 30 and last 10 elements from laser scan because they are 0.0 in scan on real MiR100
            #=list(filter(lambda a: a != 0.0, scan))   # remove all 0.0 values that are at beginning and end of scan list
            scan = np.array(scan)
            scan = np.nan_to_num(scan)
            scan = np.clip(scan, data.range_min, data.range_max)
            self.f_scan = copy.deepcopy(scan.tolist())
            self.safe_to_move_front = all(i >= 0.04 for i in scan)
        else:
            pass

    def collision_callback(self,data):
        if data.states == []:
            self.collision = False
        else:
            self.collision = True
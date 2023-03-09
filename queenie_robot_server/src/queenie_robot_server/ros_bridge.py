#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from nav_msgs.msg import Path
import PyKDL
import tf2_ros
import copy
from tf_conversions import posemath
from threading import Event
import numpy as np
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2
from cv_bridge import CvBridge, CvBridgeError
import roslib
roslib.load_manifest("queenie")
import actionlib
from queenie.msg import ExploreAction, ExploreGoal
import cv2
from control_msgs.msg import JointTrajectoryControllerState

class RosBridge:

    def __init__(self, real_robot=False):

        self.camera_mode = "RGB" # POSSIBLE VALUES INCLUDE RGB, D, GD

        if self.camera_mode == "GD":
            self.camera_image_dim = 84*84*1
        elif self.camera_mode == "RGB":
            self.camera_image_dim = 84*84*3
        elif self.camera_image_dim == "RGBD":
            self.camera_image_dim = 84*84*3

        self.joint_names = ["neck", "neck_x"]
        self.joint_position = dict.fromkeys(self.joint_names, 0.0)

        traj_publisher = rospy.Publisher('/queenie/head_controller/command', JointTrajectory, queue_size=1)
        gripper_command_pub = rospy.Publisher('/queenie/gripper_controller/command', Float64MultiArray, queue_size=1)
        self.arm_cmd_pub = rospy.Publisher('env_arm_command', JointTrajectory, queue_size=1) # joint_trajectory_command_handler publishe

        


        self.laserlen = 133

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        self.bridge = CvBridge()

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

        # action server clients
        self.explore_client = actionlib.SimpleActionClient("explore", ExploreAction)
        self.explore_client.wait_for_server()

        # subscribers
        rospy.Subscriber('left_finger_contact', Bool, self.left_finger_contact_callback)
        rospy.Subscriber('right_finger_contact', Bool, self.right_finger_contact_callback)
        rospy.Subscriber('palm_contact', Bool, self.palm_contact_callback)
        rospy.Subscriber('min_distance_to_handle', Float32, self.min_distance_to_handle_callback)
        rospy.Subscriber('angle_to_handle', Float32, self.angle_to_handle_callback)
        rospy.Subscriber('handle_pc_count', Float32, self.handle_pc_count_callback)
        # rospy.Subscriber('camera/processed_laser_scan', Float64MultiArray, self.laser_scan_callback)
        rospy.Subscriber('robot_pose', Pose, self.callbackState, queue_size=1)
        rospy.Subscriber('camera/color/image_raw', Image, self.rgb_image_callback)
        rospy.Subscriber("queenie/head_controller/state", JointTrajectoryControllerState, self.on_joint_states)
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [5,5]
        for _ in range(3):
            gripper_command_pub.publish(gripper_msg)
            rospy.sleep(0.5)

        for _ in range(0, 2):
            msg = JointTrajectory()
            msg.joint_names = ["neck", "neck_x"]
            msg.points=[JointTrajectoryPoint()]
            msg.header =Header()
            msg.points[0].positions = [0, 0]
            msg.points[0].time_from_start = rospy.Duration.from_sec(0.1)
            traj_publisher.publish(msg)
            rospy.sleep(1)
        

        self.target = [0.0] * 3
        self.queenie_pose = [0.0] * 3
        self.object_pose = [0.0] * 3
        self.queenie_twist = [0.0] * 2
        self.min_distance_to_handle = [21.0]
        self.angle_to_handle = [0.0]
        self.left_finger_contact = [0.0]
        self.right_finger_contact = [0.0]
        self.palm_contact = [0.0]
        self.camera_image = [0] * self.camera_image_dim

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
        object_pose = copy.deepcopy(self.object_pose)
        queenie_twist = copy.deepcopy(self.queenie_twist)
        min_distance_to_handle = copy.deepcopy(self.min_distance_to_handle)
        angle_to_handle = copy.deepcopy(self.angle_to_handle)
        # laser_scan = copy.deepcopy(self.laser_scan)
        left_finger_contact = copy.deepcopy(self.left_finger_contact)
        right_finger_contact = copy.deepcopy(self.right_finger_contact)
        palm_contact = copy.deepcopy(self.palm_contact)
        camera_image = copy.deepcopy(self.camera_image)

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(queenie_pose)
        msg.state.extend(object_pose)
        msg.state.extend(queenie_twist)
        # msg.state.extend(visible_handle_points)
        msg.state.extend(min_distance_to_handle)
        msg.state.extend(angle_to_handle)
        # msg.state.extend(laser_scan)
        msg.state.extend(left_finger_contact)
        msg.state.extend(right_finger_contact)
        msg.state.extend(palm_contact)
        msg.state.extend(camera_image)
        msg.success = 1
        
        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state
        # Clear reset Event
        self.reset.clear()

        # Set target internal value
        self.target = copy.deepcopy(state[0:3])

        if not self.real_robot :
            # Set Gazebo Robot Model state
            self.set_model_state('queenie', copy.deepcopy(state[3:6]))
            # Set Gazebo Target Model state
            # self.set_model_state('target', copy.deepcopy(state[0:3]))
            # Set obstacles poses
            
            if self.target[0] == -1:
                self.set_model_state('large_cuboid_tilted_handle', copy.deepcopy(state[6:9]))
                rospy.sleep(0.5)


            elif self.target[0] == 1:
                
                self.set_model_state('large_cuboid_2', [-40,40,0])
                rospy.sleep(0.5)
                self.set_model_state('large_cuboid_3', [-40,50,0])
                rospy.sleep(0.5)
                self.set_model_state('large_cuboid', copy.deepcopy(state[6:9]))
            elif self.target[0] == 2:
                self.set_model_state('large_cuboid', [-40,-40,0])
                rospy.sleep(0.5)
                self.set_model_state('large_cuboid_3', [-40,50,0])
                rospy.sleep(0.5)
                self.set_model_state('large_cuboid_2', copy.deepcopy(state[6:9]))
            elif self.target[0] == 3:
                self.set_model_state('large_cuboid', [-40,-40,0])
                rospy.sleep(0.5)
                self.set_model_state('large_cuboid_2',[-40,40,0])
                rospy.sleep(0.5)
                self.set_model_state('large_cuboid_3', copy.deepcopy(state[6:9]))

        # Set reset Event
        self.reset.set()

        if not self.real_robot:
            # Sleep time set manually to allow gazebo to reposition model
            rospy.sleep(0.2)

        return 1

    def publish_env_arm_delta_cmd(self, delta_cmd):
        msg = JointTrajectory()
        msg.header = Header()
        msg.joint_names = self.joint_names
        msg.points=[JointTrajectoryPoint()]
        # msg.points[0].positions = position_cmd
        position_cmd = []
        dur = []
        for idx, name in enumerate(msg.joint_names):
            pos = self.joint_position[name]
            cmd = delta_cmd[idx]
            dur.append(0.1)
            if name == "neck":
                position_cmd.append(0)
            else:
                position_cmd.append(pos + cmd)
        msg.points[0].positions = position_cmd
        msg.points[0].time_from_start = rospy.Duration.from_sec(max(dur))
        self.arm_cmd_pub.publish(msg)
        rospy.sleep(0.07)
        return position_cmd
    
    def publish_env_cmd_vel(self, lin_vel, ang_vel):
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        self.env_cmd_vel_pub.publish(msg)
        # Sleep time set manually to achieve approximately 10Hz rate
        rospy.sleep(0.07)
        return lin_vel, ang_vel

    def send_explore_goal(self):
        goal = ExploreGoal()
        self.explore_client.send_goal(goal)
        self.explore_client.wait_for_result()
        return self.explore_client.get_result()

    

    def set_model_state(self, model_name, state):
        # Set Gazebo Model State
        rospy.wait_for_service('/gazebo/set_model_state')

        start_state = ModelState()
        start_state.model_name = model_name
        start_state.pose.position.x = state[0]
        start_state.pose.position.y = state[1]
        start_state.pose.position.z = 0.5
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
    
    def on_joint_states(self, msg:JointTrajectoryControllerState):
        if self.reset.isSet():
            for idx, name in enumerate(msg.joint_names):
                if name in self.joint_names:
                    if name == "neck":
                        self.joint_position[name] = 0
                    self.joint_position[name] = msg.actual.positions[idx]
                    # self.joint_velocity[name] = msg.velocity[idx]

    def min_distance_to_handle_callback(self, data):
        self.min_distance_to_handle = [data.data]

    def angle_to_handle_callback(self, data):
        self.angle_to_handle = [data.data - np.pi/2]
        

    def handle_pc_count_callback(self, data):
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
            x = data.position.x
            y = data.position.y

            orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                                 data.orientation.y,
                                                 data.orientation.z,
                                                 data.orientation.w)

            euler_orientation = orientation.GetRPY()
            yaw = euler_orientation[2]


            # Update internal Pose variable
            self.queenie_pose = copy.deepcopy([x, y, yaw])
        else:
            pass
    
    
    def left_finger_contact_callback(self, data):
        self.left_finger_contact = copy.deepcopy([data.data])
    
    def right_finger_contact_callback(self, data):
        self.right_finger_contact = copy.deepcopy([data.data])
    
    def palm_contact_callback(self, data):
        self.palm_contact = copy.deepcopy([data.data])

    def rgb_image_callback(self, data):
        if self.reset.isSet():
            try:
                # Convert ROS image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                cv_image = np.transpose(cv_image, (2, 0, 1))

            except CvBridgeError as e:
                print(e)
                return
            if self.camera_mode == "GD":
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.camera_image = cv_image.flatten()
        

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
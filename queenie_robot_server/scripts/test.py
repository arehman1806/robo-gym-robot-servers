from queenie_robot_server.ros_bridge import RosBridge
import rospy
rospy.init_node("abc")

x = RosBridge()

rospy.spin()
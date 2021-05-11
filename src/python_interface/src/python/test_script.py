#!/usr/bin/env python3
from _core import *
# test pick.py https://github.com/ros-planning/moveit2/blob/main/moveit_commander/demos/pick.py
import sys
import rclpy
from rclpy.node import Node
from robot import RobotCommander
from geometry_msgs.msg import PoseStamped

# Todo:
# 1. rclcpp_init()'s problem: use boost::python::dict& remappings & rclcpp::executors
# 2. rewrite planning_sence_interface.py
# 3. rewrite robot.py
# 4. test pick.py (https://github.com/ros-planning/moveit2/blob/main/moveit_commander/demos/pick.py)
def main(args = None):
    rclcpp_init(sys.argv)
    rclpy.init(args = args)
    node = Node('moveit_py_demo')
    node.get_logger().info('moveit2 python interface demo')
    rclcpp_shutdown()

if __name__ == "__main__":
    main()
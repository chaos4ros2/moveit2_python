#!/usr/bin/env python3
from _core import *
# test pick.py https://github.com/ros-planning/moveit2/blob/main/moveit_commander/demos/pick.py
import sys
import rclpy
from moveit_commander import (
    # RobotCommander,
    # PlanningSceneInterface,
)
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rclcpp_init()
    rclcpp_shutdown()
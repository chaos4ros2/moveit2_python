#!/usr/bin/env python3
import rclpy
import rclpy.node import Node
from rosgraph.names import ns_join
from . import conversions

from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_ros_planning_interface import _moveit_planning_scene_interface
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from .exception import MoveItCommanderException
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest

class PlanningSceneInterface(Node):
    """ Simple interface to making updates to a planning scene """

    def __init__(self, ns="", synchronous=False, service_timeout=5.0):
        """ Create a planning scene interface; it uses both C++ wrapped methods and scene manipulation topics. """
        self._pub_co = rospy.Publisher(
            ns_join(ns, "collision_object"), CollisionObject, queue_size=100
        )
        self._pub_aco = rospy.Publisher(
            ns_join(ns, "attached_collision_object"),
            AttachedCollisionObject,
            queue_size=100,
        )
        self.__synchronous = synchronous
        if self.__synchronous:
            self._apply_planning_scene_diff = rospy.ServiceProxy(
                ns_join(ns, "apply_planning_scene"), ApplyPlanningScene
            )
            self._apply_planning_scene_diff.wait_for_service(service_timeout)

    def __submit(self, collision_object, attach=False):
        if self.__synchronous:
            diff_req = self.__make_planning_scene_diff_req(collision_object, attach)
            self._apply_planning_scene_diff.call(diff_req)
        else:
            if attach:
                self._pub_aco.publish(collision_object)
            else:
                self._pub_co.publish(collision_object)

    def add_box(self, name, pose, size=(1, 1, 1)):
        """
        Add a box to the planning scene
        """
        co = self.__make_box(name, pose, size)
        self.__submit(co, attach=False)

    def remove_world_object(self, name=None):
        """
        Remove an object from planning scene, or all if no name is provided
        """
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        if name is not None:
            co.id = name
        self.__submit(co, attach=False)

    @staticmethod
    def __make_box(name, pose, size):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]
        return co

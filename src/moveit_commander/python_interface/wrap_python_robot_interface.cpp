/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit/py_bindings_tools/gil_releaser.h>
#include <moveit_msgs/msg/robot_state.hpp>
// ####################################################################
// # old
// #include <visualization_msgs/MarkerArray.h>
// ####################################################################
// # new 425
#include <visualization_msgs/msg/marker_array.hpp>
// ####################################################################

#include <stdexcept>
#include <boost/python.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;
using moveit::py_bindings_tools::GILReleaser;

namespace moveit
{
class RobotInterfacePython : protected py_bindings_tools::ROScppInitializer
{
public:
  RobotInterfacePython(const std::string& robot_description, const std::string& ns = "")
    : py_bindings_tools::ROScppInitializer()
  {
    // #############################################################
    // old
    // robot_model_ = planning_interface::getSharedRobotModel(robot_description);
    // #############################################################
    // new 425 add node to match arguments size
    auto node = rclcpp::Node::make_shared("moveit_python_wrappers");
    robot_model_ = planning_interface::getSharedRobotModel(node, robot_description);
    // ##############################################################
    if (!robot_model_)
      throw std::runtime_error("RobotInterfacePython: invalid robot model");
    current_state_monitor_ =
        // ##############################################################################################
        // old
        // planning_interface::getSharedStateMonitor(robot_model_, planning_interface::getSharedTF(), ns);
        // ##############################################################################################
        // new
        planning_interface::getSharedStateMonitor(node, robot_model_, planning_interface::getSharedTF());
        // ##############################################################################################
  }



  const char* getPlanningFrame() const
  {
    return robot_model_->getModelFrame().c_str();
  }

  
private:
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  //########################
  // # old not use in anywhere, commentout temporarily
  // https://answers.ros.org/question/295701/get-ros-2-nodehandle-in-case-of-composed-nodes/
  // https://qiita.com/wacatsuki/items/95e39fdc8b409e84498d
  // ros::NodeHandle nh_;
  // #######################
};
}  // namespace moveit

static void wrap_robot_interface()
{
  using namespace moveit;

  boost::python::class_<RobotInterfacePython> robot_class("RobotInterface", boost::python::init<std::string, boost::python::optional<std::string>>());
  robot_class.def("get_planning_frame", &RobotInterfacePython::getPlanningFrame);
}

BOOST_PYTHON_MODULE(_moveit_robot_interface)
{
  wrap_robot_interface();
}

/** @endcond */

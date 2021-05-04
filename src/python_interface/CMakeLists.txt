cmake_minimum_required(VERSION 3.5)
project(python_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
# find_package(moveit_task_constructor_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(rviz_marker_tools REQUIRED)

# ament_target_dependencies(cpp_test
#  "rclcpp"
# )


set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
  common_planning_interface_objects/include
  planning_scene_interface/include
  move_group_interface/include
  moveit_cpp/include
  ${PYTHON_INCLUDE_PATH}
)

add_library(_core SHARED
	src/python/core.cpp
	src/python/ros_init.cpp
	src/python/task.cpp
)

target_link_libraries(_core
  ${PROJECT_NAME} ${PROJECT_NAME}_stages ${PYTHON_LIBRARIES} ${Boost_LIBRARIES}
 )
set_target_properties(_core PROPERTIES PREFIX "")

# Install cpp executables
# install(TARGETS
#  cpp_test
#  DESTINATION lib/${PROJECT_NAME}
# )


# Install python scripts
install(PROGRAMS
  src/python/test_script.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install share library
install(TARGETS _core
	ARCHIVE DESTINATION lib/${PROJECT_NAME}
	LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()

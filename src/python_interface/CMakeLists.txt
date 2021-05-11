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
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
# find_package(geometry_msgs REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
  ${PYTHON_INCLUDE_PATH}
  # common_planning_interface_objects/include
  # planning_scene_interface/include
  # move_group_interface/include
  # moveit_cpp/include
)

add_library(_core SHARED
	src/python/core.cpp
	src/python/ros_init.cpp
)

ament_target_dependencies(_core
  "rclcpp"
)

target_link_libraries(_core
  ${PYTHON_LIBRARIES} ${Boost_LIBRARIES}
  # ${PROJECT_NAME} ${PROJECT_NAME}_stages ${PYTHON_LIBRARIES} ${Boost_LIBRARIES}
)

set_target_properties(_core PROPERTIES PREFIX "")

# Install python scripts
install(PROGRAMS
  src/python/test_script.py
  src/python/moveit_commander/robot.py
  src/python/moveit_commander/planning_scene_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install share library
install(TARGETS _core
	ARCHIVE DESTINATION lib/${PROJECT_NAME}
	LIBRARY DESTINATION lib/${PROJECT_NAME}
)

ament_package()

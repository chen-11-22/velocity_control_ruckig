cmake_minimum_required(VERSION 3.10)
project(robot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)

LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-2.8/Modules")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ruckig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tm_msgs REQUIRED)
find_package(tm_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include ${EIGEN3_INCLUDE_DIR}
  include include/
)

# link_directories(/home/ubuntu20auo/Documents/NYCU-Grinding/install/tm_kinematics/lib)

# add_executable(gazebo_control src/gazebo_control.cpp)
# target_compile_features(gazebo_control PUBLIC cxx_std_17)
# target_link_libraries(gazebo_control ruckig::ruckig)
# ament_target_dependencies(gazebo_control 
#   rclcpp
#   std_msgs
#   sensor_msgs
#   tm_msgs
#   Eigen3
#   tm_kinematics
# )
# install(TARGETS
# gazebo_control
#   DESTINATION lib/${PROJECT_NAME})


# add_executable(pub_tool0_state src/pub_tool0_state.cpp)
# ament_target_dependencies(pub_tool0_state 
#   rclcpp
#   std_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
# )
# install(TARGETS
# pub_tool0_state
#   DESTINATION lib/${PROJECT_NAME})

# add_executable(gazebo_controller src/gazebo_controller.cpp)
# target_compile_features(gazebo_controller PUBLIC cxx_std_17)
# target_link_libraries(gazebo_controller ruckig::ruckig)
# ament_target_dependencies(gazebo_controller 
#   rclcpp
#   std_msgs
#   sensor_msgs
#   tm_msgs
#   Eigen3
#   tm_kinematics
# )
# install(TARGETS
# gazebo_controller
# DESTINATION lib/${PROJECT_NAME})

add_executable(polish_controller src/polish_controller.cpp)
target_compile_features(polish_controller PUBLIC cxx_std_17)
target_link_libraries(polish_controller ruckig::ruckig)
ament_target_dependencies(polish_controller 
  rclcpp
  std_msgs
  sensor_msgs
  tm_msgs
  Eigen3
  tm_kinematics
)
install(TARGETS
polish_controller
DESTINATION lib/${PROJECT_NAME})

# add_executable(joint2cart src/joint2cart.cpp)
# target_compile_features(joint2cart PUBLIC cxx_std_17)
# ament_target_dependencies(joint2cart 
#   rclcpp
#   Eigen3
#   tm_kinematics
# )
# install(TARGETS
# joint2cart
# DESTINATION lib/${PROJECT_NAME})

# add_executable(interface src/interface.cpp)
# target_compile_features(interface PUBLIC cxx_std_17)
# ament_target_dependencies(interface 
#   rclcpp
#   std_msgs
#   sensor_msgs
#   tm_msgs
#   Eigen3
#   tm_kinematics
# )
# install(TARGETS
# interface
# DESTINATION lib/${PROJECT_NAME})

add_executable(sim_arm_controller src/sim_arm_controller.cpp)
target_compile_features(sim_arm_controller PUBLIC cxx_std_17)
target_link_libraries(sim_arm_controller ruckig::ruckig)
ament_target_dependencies(sim_arm_controller 
  rclcpp
  std_msgs
  sensor_msgs
  tm_msgs
  Eigen3
  tm_kinematics
)
install(TARGETS
sim_arm_controller
DESTINATION lib/${PROJECT_NAME})

ament_package()
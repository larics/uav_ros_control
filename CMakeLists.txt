cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_control)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             uav_ros_msgs
             dynamic_reconfigure
             uav_ros_lib
             mavros_msgs
             uav_ros_msgs
             pluginlib)

generate_dynamic_reconfigure_options(
  config/control/PositionControlParameters.cfg
  config/control/DistanceControlParameters.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  uav_pid
  uav_util
  CATKIN_DEPENDS
  uav_ros_msgs)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

# PID controller library
add_library(uav_pid src/control/PID.cpp)
target_link_libraries(uav_pid ${catkin_LIBRARIES})

# uav_util library
add_library(uav_util src/util/uav_util.cpp src/util/AutomaticTakeoff.cpp)
target_link_libraries(uav_util ${catkin_LIBRARIES})

# References library
file(GLOB_RECURSE reference_SRC "src/reference/*.cpp")
add_library(reference_lib ${reference_SRC})
target_link_libraries(reference_lib ${catkin_LIBRARIES} uav_pid yaml-cpp)

# Carrot node
add_executable(carrot_reference_node nodes/reference/CarrotReferenceNode.cpp)
target_link_libraries(carrot_reference_node reference_lib)

# PID cascade node
add_executable(
  pid_cascade_node nodes/control/PositionControlNode.cpp
                   src/control/CascadePID.cpp src/control/ControlBase.cpp)
target_link_libraries(pid_cascade_node uav_pid)
add_dependencies(pid_cascade_node ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

add_executable(
  pid_cascade_node_yawrate
  nodes/control/PositionControlNode_yawrate.cpp src/control/CascadePID.cpp
  src/control/ControlBase.cpp)
target_link_libraries(pid_cascade_node_yawrate uav_pid)
add_dependencies(pid_cascade_node_yawrate ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

add_executable(
  pid_cascade_node_px4 nodes/control/PositionControlNode_px4.cpp
                       src/control/CascadePID.cpp src/control/ControlBase.cpp)
target_link_libraries(pid_cascade_node_px4 uav_pid)
add_dependencies(pid_cascade_node_px4 ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

add_executable(
  pid_cascade_node_px4_yawrate
  nodes/control/PositionControlNode_px4_yawrate.cpp src/control/CascadePID.cpp
  src/control/ControlBase.cpp)
target_link_libraries(pid_cascade_node_px4_yawrate uav_pid)
add_dependencies(pid_cascade_node_px4_yawrate ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

# Distance control node
add_executable(
  distance_control_node
  nodes/control/DistanceControlNode.cpp src/control/CascadePID.cpp
  src/control/DistanceController.cpp src/control/ControlBase.cpp)
target_link_libraries(distance_control_node ${catkin_LIBRARIES} uav_pid)
add_dependencies(distance_control_node ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

# Geo-fencing node
add_executable(geo_fence_node nodes/reference/GeoFenceNode.cpp)
target_link_libraries(geo_fence_node reference_lib ${catkin_LIBRARIES})

# Model Predictive Control library
add_library(uav_mpc src/control/ModelPredictiveControl.cpp)
target_link_libraries(uav_mpc ${catkin_LIBRARIES} control_cvx_wrapper)

# CVXGEN MPC solver
add_library(
  control_mpc_cvxgen
  src/control/cvxgen/ldl.c src/control/cvxgen/matrix_support.c
  src/control/cvxgen/solver.c src/control/cvxgen/util.c)

add_library(control_cvx_wrapper src/control/cvx_wrapper.cpp)
target_link_libraries(control_cvx_wrapper control_mpc_cvxgen)

# Control manager node
add_executable(control_manager nodes/control/ControlManagerNode.cpp)
target_link_libraries(control_manager ${catkin_LIBRARIES})

install(
  TARGETS uav_mpc pid_cascade_node carrot_reference_node distance_control_node
          geo_fence_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

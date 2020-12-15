cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_control)

find_package(catkin REQUIRED COMPONENTS roscpp uav_ros_msgs dynamic_reconfigure
                                        mavros)

generate_dynamic_reconfigure_options(config/control/PositionControlParameters.cfg
                                     config/control/DistanceControlParameters.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  uav_pid
  nfilt
  uav_kalman
  CATKIN_DEPENDS
  uav_ros_msgs
  mavros)

include_directories(include ${catkin_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS})

# PID controller library
add_library(uav_pid src/control/PID.cpp)
target_link_libraries(uav_pid ${catkin_LIBRARIES})

# Nonlinear filters library
add_library(nfilt src/filters/NonlinearFilters.cpp)
target_link_libraries(nfilt ${catkin_LIBRARIES})

# Kalman filter library
add_library(uav_kalman src/filters/KalmanFilter.cpp)
target_link_libraries(uav_kalman ${catkin_LIBRARIES})

# References library
file(GLOB_RECURSE reference_SRC "src/reference/*.cpp")
add_library(reference_lib ${reference_SRC})
target_link_libraries(reference_lib ${catkin_LIBRARIES} nfilt uav_pid yaml-cpp)

# Controller library
file(GLOB_RECURSE controllers_SRC "src/control/*.cpp")
add_library(control_lib ${controllers_SRC})
target_link_libraries(control_lib ${catkin_LIBRARIES} nfilt)

# Carrot node
add_executable(carrot_reference_node nodes/reference/CarrotReferenceNode.cpp)
target_link_libraries(carrot_reference_node reference_lib)

# PID cascade node
add_executable(pid_cascade_node nodes/control/PositionControlNode.cpp)
target_link_libraries(pid_cascade_node control_lib)

# Distance control node
add_executable(distance_control_node nodes/control/DistanceControlNode.cpp)
target_link_libraries(distance_control_node control_lib nfilt)

# Geo-fencing node
add_executable(geo_fence_node nodes/reference/GeoFenceNode.cpp)
target_link_libraries(geo_fence_node reference_lib ${catkin_LIBRARIES} nfilt)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

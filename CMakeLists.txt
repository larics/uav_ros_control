cmake_minimum_required(VERSION 3.10.0)
project(uav_ros_control)

find_package(catkin REQUIRED COMPONENTS roscpp uav_ros_msgs dynamic_reconfigure
                                        uav_ros_lib)

generate_dynamic_reconfigure_options(config/control/PositionControlParameters.cfg
                                     config/control/DistanceControlParameters.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  uav_pid
  CATKIN_DEPENDS
  uav_ros_msgs)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

# PID controller library
add_library(uav_pid src/control/PID.cpp)
target_link_libraries(uav_pid ${catkin_LIBRARIES})

# References library
file(GLOB_RECURSE reference_SRC "src/reference/*.cpp")
add_library(reference_lib ${reference_SRC})
target_link_libraries(reference_lib ${catkin_LIBRARIES} uav_pid yaml-cpp)

# Controller library
file(GLOB_RECURSE controllers_SRC "src/control/*.cpp")
add_library(control_lib ${controllers_SRC})
target_link_libraries(control_lib ${catkin_LIBRARIES})
add_dependencies(control_lib ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Carrot node
add_executable(carrot_reference_node nodes/reference/CarrotReferenceNode.cpp)
target_link_libraries(carrot_reference_node reference_lib)

# PID cascade node
add_executable(pid_cascade_node nodes/control/PositionControlNode.cpp)
target_link_libraries(pid_cascade_node control_lib)

# Distance control node
add_executable(distance_control_node nodes/control/DistanceControlNode.cpp)
target_link_libraries(distance_control_node control_lib)

# Geo-fencing node
add_executable(geo_fence_node nodes/reference/GeoFenceNode.cpp)
target_link_libraries(geo_fence_node reference_lib ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

cmake_minimum_required(VERSION 3.0.2)
project(dt_test_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  # serial
)
find_package(Eigen3 REQUIRED)


add_message_files(
  FILES
  Wind.msg
)


add_service_files(
  FILES
  ControlEval.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs#   
  sensor_msgs#   
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dt_test_pkg
  CATKIN_DEPENDS geometry_msgs mavros roscpp rospy sensor_msgs std_msgs std_srvs
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(environment
  src/env_node.cpp
)
target_link_libraries(environment
  ${catkin_LIBRARIES}
)

add_executable(digital_twin
  src/digital_twin.cpp
)
target_link_libraries(digital_twin
  ${catkin_LIBRARIES}
)

add_executable(physical_entity
  src/physical_entity.cpp
)
target_link_libraries(physical_entity
  ${catkin_LIBRARIES}
)

add_executable(flight_square
  src/flght_square.cpp
)
target_link_libraries(flight_square
  ${catkin_LIBRARIES}
)
add_executable(flight_hover_20sec
  src/flight_hover_20sec.cpp
)
target_link_libraries(flight_hover_20sec
  ${catkin_LIBRARIES}
)
add_executable(flight_spin
  src/flight_spin.cpp
)
target_link_libraries(flight_spin
  ${catkin_LIBRARIES}
)

add_executable(flight_swing
  src/flight_swing.cpp
)
target_link_libraries(flight_swing
  ${catkin_LIBRARIES}
)

add_executable(flight_4real_square
  src/flight_4real_square.cpp
)
target_link_libraries(flight_4real_square
  ${catkin_LIBRARIES}
)

add_executable(flight_4real_swing_x
  src/flight_4real_swing_x.cpp
)
target_link_libraries(flight_4real_swing_x
  ${catkin_LIBRARIES}
)

add_executable(flight_4real_swing_y
  src/flight_4real_swing_y.cpp
)
target_link_libraries(flight_4real_swing_y
  ${catkin_LIBRARIES}
)


add_executable(flight_4real_spin
  src/flight_4real_spin.cpp
)
target_link_libraries(flight_4real_spin
  ${catkin_LIBRARIES}
)


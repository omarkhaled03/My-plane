cmake_minimum_required(VERSION 2.8.3)
project(fixedwing)



find_package( catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    message_generation
    gazebo_plugins
    gazebo_ros
    dynamic_reconfigure
)
find_package(gazebo REQUIRED) # for Gazebo

link_directories(${GAZEBO_LIBRARY_DIRS})# for Gazebo

generate_dynamic_reconfigure_options(
  cfg/controller.cfg
  cfg/follower.cfg
)


add_message_files(
    FILES
    state.msg
    EOM.msg
    FAM.msg
    command.msg
    input.msg
    controller_command.msg
    controller_internals.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
    roscpp
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")# for Gazebo

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include # for include in package 
  ${catkin_INCLUDE_DIRS}# for include in build
  ${GAZEBO_INCLUDE_DIRS} # for Gazebo
)


#add_executable(controller
#    src/Controller.cpp
#)

add_executable(talker
    src/Take_off.cpp
)
#target_link_libraries(controller  ${catkin_LIBRARIES})
target_link_libraries(talker  ${catkin_LIBRARIES})
add_library(wind src/Wind.cpp)
add_library(state_plugin src/States.cpp)
add_library(EOM src/EOM.cpp)
add_library(Forces_Moments src/Forces_Moments.cpp)

target_link_libraries(state_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(wind ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(EOM ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(Forces_Moments ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#target_link_libraries(controller ${catkin_LIBRARIES})

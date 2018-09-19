cmake_minimum_required(VERSION 2.8.3)
project(robit_urdf)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(joint_state_publisher src/joint_state_publisher.cpp)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES})
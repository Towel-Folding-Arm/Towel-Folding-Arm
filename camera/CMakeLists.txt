cmake_minimum_required(VERSION 2.8.3)
project(camera)

#include(~/dlib-19.6/dlib/cmake)
add_definitions(-msse2 -pthread -O2 -std=c++11 -fPIC -ffast-math -fpermissive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)


find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
find_package(OpenCV REQUIRED)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp sensor_msgs
   DEPENDS eigen opencv
)

message("catkin_INCLUDE_DIRS=" ${catkin_INCLUDE_DIRS})
message("INCLUDE_DIRS=" ${INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS}) #fake dlib

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN_INCLUDE_DIRS}
  ./include 
)

#include_directories(
#    ${Boost_INCLUDE_DIRS} # dlib
#    ${GSTREAMER_INCLUDE_DIRS} # dlib
#)

#add_executable(listener2 src/listener2.cpp)

#target_link_libraries(listener2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(listener2 dlib::dlib) #dlib
#target_link_libraries(listener2 ${OpenCV_LIBS}) #dlib
#target_link_libraries(listener2 ${catkin_LIBS}) #dlib
#target_link_libraries(listener2 ${Boost_LIBS}) #dlib
#target_link_libraries(listener2 ${GSTREAMER_LIBS}) #dlib

macro(add name)
  add_executable(${name} src/${name}.cpp)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  target_link_libraries(${name} ${OpenCV_LIBS})
  target_link_libraries(${name} ${catkin_LIBS})
endmacro()

add(listener2)
add(listener3)
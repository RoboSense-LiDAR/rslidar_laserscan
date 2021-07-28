cmake_minimum_required (VERSION 2.8.3)

project (rslidar_laserscan)

# Set minimum C++ standard to C++11
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message (STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message (STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set (CMAKE_CXX_STANDARD 11)
endif()

find_package (catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories (
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable (${PROJECT_NAME}_node
  src/node.cpp src/rslidar_laserscan.cpp
)

target_link_libraries (${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

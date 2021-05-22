cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
endif()
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common 
PUBLIC Franka::Franka 
PUBLIC ${Eigen_INCLUDE_DIRS} 
PUBLIC ${ROS_LIBRARIES})

set(EXAMPLES
  joint_trajectory_pub
  joint_trajectory_sub
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp spline.h )
  target_link_libraries(${example} Franka::Franka examples_common  ${Eigen_INCLUDE_DIRS} ${catkin_LIBRARIES}  ${ROS_LIBRARIES})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  
)
target_include_directories(${example} PUBLIC  ${catkin_INCLUDE_DIRS} )
endforeach()

#target_link_libraries(joint_impedance_control Threads::Threads)
#target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

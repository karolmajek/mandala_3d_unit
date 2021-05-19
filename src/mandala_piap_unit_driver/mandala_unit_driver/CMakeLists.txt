cmake_minimum_required(VERSION 2.8.3)
project(mandala_unit_driver)


  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=gnu++14 ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 14)
  endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  tf
  message_generation
  mandala_unit_msgs
)
find_package(PCL 1.5 REQUIRED)
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   encoder_stamped.msg
   )
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
 )
catkin_package(
#  LIBRARIES 3dunit_develop_driver
   CATKIN_DEPENDS roscpp tf pcl_ros tf_conversions  message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
#########################################################
# MAVLINK
#########################################################
SET(_MAVLINK_HEADERS mavlink/3dunit)
#########################################################
# FIND BOOST
#########################################################
find_package(Boost COMPONENTS system thread filesystem date_time REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS} hpp ${PROJECT_BINARY_DIR}  ${_MAVLINK_HEADERS}   ${OpenCV_INCLUDE_DIRS} include )
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)


add_executable(encoder_udp_node src_exe/encoder_udp_node.cpp src/mavlink_client.cpp src/mavlink_client_udp.cpp)
add_dependencies(encoder_udp_node mandala_unit_develop_driver_generate_messages_cpp)
target_link_libraries(encoder_udp_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_executable(velodyne_synchro src_exe/velodyne_synchro.cpp )
add_dependencies(velodyne_synchro mandala_unit_develop_driver_generate_messages_cpp)
target_link_libraries(velodyne_synchro ${Boost_LIBRARIES} ${catkin_LIBRARIES})




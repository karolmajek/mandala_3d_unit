cmake_minimum_required(VERSION 2.8.3)
project(mandala_unit_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs sensor_msgs
)
 add_message_files(
   FILES
   encoder_stamped.msg
 )
 add_service_files(
   FILES
   cameraTrigger.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs sensor_msgs
 )
catkin_package(
#  LIBRARIES 3dunit_develop_driver
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

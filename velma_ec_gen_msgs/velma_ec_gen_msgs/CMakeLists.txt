cmake_minimum_required(VERSION 2.8.3)
project(velma_ec_gen_msgs)

find_package(catkin REQUIRED COMPONENTS
 rtt_ros
 message_generation
 rtt_subsystem_buffers
 common_interfaces
)

include_directories(${catkin_INCLUDE_DIRS})

generate_msgs_from_ec_config(velma_eni.xml)

add_message_files(
  BASE_DIR ${MESSAGE_DIR}
  FILES ${OUTPUT_FILES})

generate_messages()

catkin_package()

#LIBRARIES ${PROJECT_NAME}-ec_msg_converter)
#add_file_dependencies(${EC_MSG_CONVERTER_FILES} ${ALL_GEN_OUTPUT_FILES_cpp})
#add_library(${PROJECT_NAME}-ec_msg_converter ${EC_MSG_CONVERTER_FILES})
#target_link_libraries(${PROJECT_NAME}-ec_msg_converter ${catkin_LIBRARIES})



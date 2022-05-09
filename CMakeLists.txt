cmake_minimum_required(VERSION 2.8.3)
project(lms1xx)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(LMS1xx src/LMS1xx.cpp)
target_link_libraries(LMS1xx ${console_bridge_LIBRARIES})

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs 
  tf2 tf2_ros laser_geometry tf2_geometry_msgs tf message_generation
  std_msgs
)

add_message_files(
  FILES
  ScanIterator.msg
)

generate_messages (
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(LMS1xx_node src/LMS1xx_node.cpp)
target_link_libraries(LMS1xx_node LMS1xx ${catkin_LIBRARIES})

add_executable(Listener src/Listener.cpp)
target_link_libraries(Listener ${catkin_LIBRARIES})
add_dependencies(Listener custom_msg_generate_messages_cpp)

add_executable(scanProcessor src/scanProcessor.cpp)
target_link_libraries(scanProcessor ${catkin_LIBRARIES})
target_link_libraries(scanProcessor LMS1xx ${catkin_LIBRARIES})

#add_executable(talker src/talker_prueba.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})

#add_executable(subscriber src/subscriber_prueba.cpp)
#target_link_libraries(subscriber ${catkin_LIBRARIES})

install(TARGETS LMS1xx LMS1xx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS LMS1xx scanProcessor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/find_sick scripts/set_sick_ip
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_buffer test/test_buffer.cpp)
  target_link_libraries(test_buffer ${catkin_LIBRARIES})

  find_package(roslint REQUIRED)
  roslint_cpp()
  #roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/LMS1xx.launch)
endif()


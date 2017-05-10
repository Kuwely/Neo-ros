cmake_minimum_required(VERSION 2.8.3)
project(neo_ros)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

ADD_EXECUTABLE(neo_node src/neo_node.cpp
	src/neo_driver.cpp
	src/serial.cpp
	src/unix.cpp
	src/util.cpp)

TARGET_LINK_LIBRARIES(neo_node ${catkin_LIBRARIES})

install(TARGETS neo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

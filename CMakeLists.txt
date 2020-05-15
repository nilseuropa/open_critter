cmake_minimum_required(VERSION 2.8.3)
project(open_critter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

add_executable(gait_controller src/gait_controller.cpp)
target_link_libraries(gait_controller
  ${catkin_LIBRARIES}
)

add_executable(robot_bridge src/robot_bridge.cpp)
target_link_libraries(robot_bridge
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# install(TARGETS gait_controller
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

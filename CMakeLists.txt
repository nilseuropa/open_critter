cmake_minimum_required(VERSION 2.8.3)
project(open_critter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories()

#############
## Install ##
#############

# install(DIRECTORY launch
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(DIRECTORY params
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(DIRECTORY worlds
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

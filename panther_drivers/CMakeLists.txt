cmake_minimum_required(VERSION 2.8.3)
project(panther_drivers)

find_package(catkin REQUIRED COMPONENTS rospy rosservice)

catkin_package()

roslaunch_add_file_check(launch)

include_directories()

catkin_install_python(PROGRAMS scripts/zed_to_pose_estimate.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install param and launch folders
install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

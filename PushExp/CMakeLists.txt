cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#ADD_CUSTOM_COMMAND(
#  OUTPUT gl_parameter.h
#  COMMAND cp ${PROJECT_SOURCE_DIR}/src/gl_parameter.h
#             ${PROJECT_SOURCE_DIR}/include/PushExp/gl_parameter.h
#  DEPENDS ${PROJECT_SOURCE_DIR}/src/gl_parameter.h 
#  )

#ADD_CUSTOM_TARGET(makeH ALL DEPENDS gl_parameter.h)

#common commands for building c++ executables and libraries
rosbuild_add_library(push_action src/PushGenerator.cpp)
rosbuild_add_library(push_reset src/PushReset.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread posix_time)
rosbuild_add_executable(push_exp src/push_exp.cpp)
target_link_libraries(push_exp push_action)

rosbuild_add_executable(test_pg src/test_pg.cpp)
target_link_libraries(test_pg push_action)

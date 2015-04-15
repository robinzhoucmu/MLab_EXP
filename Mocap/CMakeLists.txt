cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# include_directories(${PROJECT_SOURCE_DIR}/include)
# Include NatNetLinux Driver.
INCLUDE_DIRECTORIES( "./NatNetLinux-master/include" )
OPTION( BUILD_EXAMPLES "If on, build executable examples." ON )
IF( ${BUILD_EXAMPLES} )
   FIND_PACKAGE( Boost COMPONENTS program_options system thread REQUIRED)
   INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )
ENDIF()
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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

ADD_CUSTOM_COMMAND(
  OUTPUT mocap_comm.h
  COMMAND cp ${PROJECT_SOURCE_DIR}/src/mocap_comm.h
             ${PROJECT_SOURCE_DIR}/include/Mocap/mocap_comm.h
  DEPENDS ${PROJECT_SOURCE_DIR}/src/mocap_comm.h 
  )

ADD_CUSTOM_TARGET(makeH ALL DEPENDS mocap_comm.h)

rosbuild_add_library(mocap_comm_lib src/mocap_comm.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_add_executable(publisher src/publisher.cpp)
rosbuild_link_boost(publisher thread program_options system)

rosbuild_add_executable(calibration_mocap src/calibration_mocap.cpp)
rosbuild_add_executable(mocap_comm src/mocap_comm.cpp)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



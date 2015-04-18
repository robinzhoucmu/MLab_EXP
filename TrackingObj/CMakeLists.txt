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

ADD_CUSTOM_COMMAND(
  OUTPUT push_obj.h
  COMMAND cp ${PROJECT_SOURCE_DIR}/src/push_obj.h
             ${PROJECT_SOURCE_DIR}/include/TrackingObj/push_obj.h
  DEPENDS ${PROJECT_SOURCE_DIR}/src/push_obj.h 
  )

ADD_CUSTOM_TARGET(makeH ALL DEPENDS push_obj.h)




#common commands for building c++ executables and libraries
rosbuild_add_library(push_obj 
                     src/push_obj.cpp
		     src/obj_reg.cpp)

rosbuild_add_library(obj_reg src/obj_reg.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(demo_obj_reg src/demo_obj_reg.cpp)
target_link_libraries(demo_obj_reg obj_reg)

rosbuild_add_executable(test_push_object src/test_push_object.cpp)
target_link_libraries(test_push_object push_obj obj_reg)

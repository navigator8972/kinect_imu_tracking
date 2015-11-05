cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#for libfreenet
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH})

rosbuild_init()
#for opengl
find_package(OpenGL)
find_package(GLUT)
#for opencv2
find_package(OpenCV REQUIRED)
#for libusb-1.0
find_package(libusb-1.0 REQUIRED)
#for libfreenect
find_package(Libfreenect REQUIRED)

include_directories(${OpenGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBUSB_1_INCLUDE_DIRS})
include_directories(${Freenect_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})

#use OpenGL
#rosbuild_add_executable(kinect_ir_main src/kinect_ir_node_gl.cpp)
#target_link_libraries(kinect_ir_main ${OpenCV_LIBRARIES} ${Freenect_LIBRARIES} ${OpenGL_LIBRARIES} ${GLUT_LIBRARY})

#use OpenCV
rosbuild_add_executable(kinect_ir_main src/kinect_ir_node_cv.cpp)
target_link_libraries(kinect_ir_main ${OpenCV_LIBRARIES} ${Freenect_LIBRARIES})

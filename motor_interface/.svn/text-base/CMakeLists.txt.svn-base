cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# Set the name to use for the executable.
set (BINNAME motorInterface)
# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set (SRCS ${SRCS} src/motorInterface.cpp)
set (SRCS ${SRCS} src/motorInterface_ROS.cpp)
set (SRCS ${SRCS} src/serial_SR.cpp)

# Set the directories where include files can be found.
include_directories (include)

# Build the executable that will be used to run this node.
rosbuild_add_executable (${BINNAME} ${SRCS})

cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize the ROS build system.
rosbuild_init ()

# Set the default path for built executables to the "bin" directory.
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Set the name to use for the executable.
set (BINNAME jsp)

# Set the source files to use with the executable.
set (SRCS ${SRCS} src/jsp.cpp)
set (SRCS ${SRCS} src/jsp_core.cpp)

# Set the directories where include files can be found.
include_directories (include)

# Build the executable that will be used to run this node.
rosbuild_add_executable (${BINNAME} ${SRCS})

# Add dynamic reconfigure API.
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

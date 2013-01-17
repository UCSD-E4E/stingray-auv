cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Name the project.
project (usb1208ls)

# Add compiler flags.
add_definitions (-Wall -O2 -g)

# Make sure the compiler can find the include files.
include_directories (include)

# Make sure the compiler can find the libraries.
link_directories (${PROJECT_BINARY_DIR})

# List the source files here.
set (SRCS ${SRCS} src/usb-1208LS)
set (SRCS ${SRCS} src/pmd)

# List the libraries here.
set (LIBS ${LIBS} mcchid)

# Build the library.
#if (BUILD_SHARED_LIBS)
add_library (${PROJECT_NAME} SHARED ${SRCS})
#else (BUILD_SHARED_LIBS)
add_library (${PROJECT_NAME} ${SRCS})
#endif (BUILD_SHARED_LIBS)

rosbuild_add_executable(usb1208ls_node usb1208ls_node/usb1208ls_node.cpp)
target_link_libraries (usb1208ls_node usb1208ls)

# Link to libraries.
target_link_libraries (usb1208ls_node ${LIBS})



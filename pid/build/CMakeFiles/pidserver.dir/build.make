# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/antonella/fuerte_workspace/sandbox/stingray/pid

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/antonella/fuerte_workspace/sandbox/stingray/pid/build

# Include any dependencies generated for this target.
include CMakeFiles/pidserver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pidserver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pidserver.dir/flags.make

CMakeFiles/pidserver.dir/src/pid.o: CMakeFiles/pidserver.dir/flags.make
CMakeFiles/pidserver.dir/src/pid.o: ../src/pid.cpp
CMakeFiles/pidserver.dir/src/pid.o: ../manifest.xml
CMakeFiles/pidserver.dir/src/pid.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/pidserver.dir/src/pid.o: /opt/ros/fuerte/share/roscpp/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/antonella/fuerte_workspace/sandbox/stingray/pid/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pidserver.dir/src/pid.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/pidserver.dir/src/pid.o -c /home/antonella/fuerte_workspace/sandbox/stingray/pid/src/pid.cpp

CMakeFiles/pidserver.dir/src/pid.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pidserver.dir/src/pid.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/antonella/fuerte_workspace/sandbox/stingray/pid/src/pid.cpp > CMakeFiles/pidserver.dir/src/pid.i

CMakeFiles/pidserver.dir/src/pid.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pidserver.dir/src/pid.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/antonella/fuerte_workspace/sandbox/stingray/pid/src/pid.cpp -o CMakeFiles/pidserver.dir/src/pid.s

CMakeFiles/pidserver.dir/src/pid.o.requires:
.PHONY : CMakeFiles/pidserver.dir/src/pid.o.requires

CMakeFiles/pidserver.dir/src/pid.o.provides: CMakeFiles/pidserver.dir/src/pid.o.requires
	$(MAKE) -f CMakeFiles/pidserver.dir/build.make CMakeFiles/pidserver.dir/src/pid.o.provides.build
.PHONY : CMakeFiles/pidserver.dir/src/pid.o.provides

CMakeFiles/pidserver.dir/src/pid.o.provides.build: CMakeFiles/pidserver.dir/src/pid.o

# Object files for target pidserver
pidserver_OBJECTS = \
"CMakeFiles/pidserver.dir/src/pid.o"

# External object files for target pidserver
pidserver_EXTERNAL_OBJECTS =

../bin/pidserver: CMakeFiles/pidserver.dir/src/pid.o
../bin/pidserver: CMakeFiles/pidserver.dir/build.make
../bin/pidserver: CMakeFiles/pidserver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/pidserver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pidserver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pidserver.dir/build: ../bin/pidserver
.PHONY : CMakeFiles/pidserver.dir/build

CMakeFiles/pidserver.dir/requires: CMakeFiles/pidserver.dir/src/pid.o.requires
.PHONY : CMakeFiles/pidserver.dir/requires

CMakeFiles/pidserver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pidserver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pidserver.dir/clean

CMakeFiles/pidserver.dir/depend:
	cd /home/antonella/fuerte_workspace/sandbox/stingray/pid/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/antonella/fuerte_workspace/sandbox/stingray/pid /home/antonella/fuerte_workspace/sandbox/stingray/pid /home/antonella/fuerte_workspace/sandbox/stingray/pid/build /home/antonella/fuerte_workspace/sandbox/stingray/pid/build /home/antonella/fuerte_workspace/sandbox/stingray/pid/build/CMakeFiles/pidserver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pidserver.dir/depend

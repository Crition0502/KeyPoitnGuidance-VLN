# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robot/test_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/test_ws/build

# Include any dependencies generated for this target.
include xfyun_kinetic/CMakeFiles/iat_node.dir/depend.make

# Include the progress variables for this target.
include xfyun_kinetic/CMakeFiles/iat_node.dir/progress.make

# Include the compile flags for this target's objects.
include xfyun_kinetic/CMakeFiles/iat_node.dir/flags.make

xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o: xfyun_kinetic/CMakeFiles/iat_node.dir/flags.make
xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o: /home/robot/test_ws/src/xfyun_kinetic/src/iat_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/test_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/iat_node.dir/src/iat_node.cpp.o -c /home/robot/test_ws/src/xfyun_kinetic/src/iat_node.cpp

xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/iat_node.dir/src/iat_node.cpp.i"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/test_ws/src/xfyun_kinetic/src/iat_node.cpp > CMakeFiles/iat_node.dir/src/iat_node.cpp.i

xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/iat_node.dir/src/iat_node.cpp.s"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/test_ws/src/xfyun_kinetic/src/iat_node.cpp -o CMakeFiles/iat_node.dir/src/iat_node.cpp.s

xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.requires:

.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.requires

xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.provides: xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.requires
	$(MAKE) -f xfyun_kinetic/CMakeFiles/iat_node.dir/build.make xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.provides.build
.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.provides

xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.provides.build: xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o


xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o: xfyun_kinetic/CMakeFiles/iat_node.dir/flags.make
xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o: /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/linuxrec.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/test_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o   -c /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/linuxrec.c

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.i"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/linuxrec.c > CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.i

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.s"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/linuxrec.c -o CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.s

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.requires:

.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.requires

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.provides: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.requires
	$(MAKE) -f xfyun_kinetic/CMakeFiles/iat_node.dir/build.make xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.provides.build
.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.provides

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.provides.build: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o


xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o: xfyun_kinetic/CMakeFiles/iat_node.dir/flags.make
xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o: /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/speech_recognizer.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/test_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o   -c /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/speech_recognizer.c

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.i"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/speech_recognizer.c > CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.i

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.s"
	cd /home/robot/test_ws/build/xfyun_kinetic && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/robot/test_ws/src/xfyun_kinetic/src/xfyun/speech_recognizer.c -o CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.s

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.requires:

.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.requires

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.provides: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.requires
	$(MAKE) -f xfyun_kinetic/CMakeFiles/iat_node.dir/build.make xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.provides.build
.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.provides

xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.provides.build: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o


# Object files for target iat_node
iat_node_OBJECTS = \
"CMakeFiles/iat_node.dir/src/iat_node.cpp.o" \
"CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o" \
"CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o"

# External object files for target iat_node
iat_node_EXTERNAL_OBJECTS =

/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: xfyun_kinetic/CMakeFiles/iat_node.dir/build.make
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/libroscpp.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/librosconsole.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/librostime.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /opt/ros/melodic/lib/libcpp_common.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node: xfyun_kinetic/CMakeFiles/iat_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/test_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable /home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node"
	cd /home/robot/test_ws/build/xfyun_kinetic && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/iat_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
xfyun_kinetic/CMakeFiles/iat_node.dir/build: /home/robot/test_ws/devel/lib/xfyun_waterplus/iat_node

.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/build

xfyun_kinetic/CMakeFiles/iat_node.dir/requires: xfyun_kinetic/CMakeFiles/iat_node.dir/src/iat_node.cpp.o.requires
xfyun_kinetic/CMakeFiles/iat_node.dir/requires: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/linuxrec.c.o.requires
xfyun_kinetic/CMakeFiles/iat_node.dir/requires: xfyun_kinetic/CMakeFiles/iat_node.dir/src/xfyun/speech_recognizer.c.o.requires

.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/requires

xfyun_kinetic/CMakeFiles/iat_node.dir/clean:
	cd /home/robot/test_ws/build/xfyun_kinetic && $(CMAKE_COMMAND) -P CMakeFiles/iat_node.dir/cmake_clean.cmake
.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/clean

xfyun_kinetic/CMakeFiles/iat_node.dir/depend:
	cd /home/robot/test_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/test_ws/src /home/robot/test_ws/src/xfyun_kinetic /home/robot/test_ws/build /home/robot/test_ws/build/xfyun_kinetic /home/robot/test_ws/build/xfyun_kinetic/CMakeFiles/iat_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : xfyun_kinetic/CMakeFiles/iat_node.dir/depend


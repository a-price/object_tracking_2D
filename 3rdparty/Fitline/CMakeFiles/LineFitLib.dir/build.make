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

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tox/git/object_tracking_2D/3rdparty/Fitline

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tox/git/object_tracking_2D/3rdparty/Fitline

# Include any dependencies generated for this target.
include CMakeFiles/LineFitLib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LineFitLib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LineFitLib.dir/flags.make

CMakeFiles/LineFitLib.dir/fitline.cpp.o: CMakeFiles/LineFitLib.dir/flags.make
CMakeFiles/LineFitLib.dir/fitline.cpp.o: fitline.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tox/git/object_tracking_2D/3rdparty/Fitline/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LineFitLib.dir/fitline.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LineFitLib.dir/fitline.cpp.o -c /home/tox/git/object_tracking_2D/3rdparty/Fitline/fitline.cpp

CMakeFiles/LineFitLib.dir/fitline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LineFitLib.dir/fitline.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tox/git/object_tracking_2D/3rdparty/Fitline/fitline.cpp > CMakeFiles/LineFitLib.dir/fitline.cpp.i

CMakeFiles/LineFitLib.dir/fitline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LineFitLib.dir/fitline.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tox/git/object_tracking_2D/3rdparty/Fitline/fitline.cpp -o CMakeFiles/LineFitLib.dir/fitline.cpp.s

CMakeFiles/LineFitLib.dir/fitline.cpp.o.requires:
.PHONY : CMakeFiles/LineFitLib.dir/fitline.cpp.o.requires

CMakeFiles/LineFitLib.dir/fitline.cpp.o.provides: CMakeFiles/LineFitLib.dir/fitline.cpp.o.requires
	$(MAKE) -f CMakeFiles/LineFitLib.dir/build.make CMakeFiles/LineFitLib.dir/fitline.cpp.o.provides.build
.PHONY : CMakeFiles/LineFitLib.dir/fitline.cpp.o.provides

CMakeFiles/LineFitLib.dir/fitline.cpp.o.provides.build: CMakeFiles/LineFitLib.dir/fitline.cpp.o

CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o: CMakeFiles/LineFitLib.dir/flags.make
CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o: LFLineFitter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tox/git/object_tracking_2D/3rdparty/Fitline/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o -c /home/tox/git/object_tracking_2D/3rdparty/Fitline/LFLineFitter.cpp

CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tox/git/object_tracking_2D/3rdparty/Fitline/LFLineFitter.cpp > CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.i

CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tox/git/object_tracking_2D/3rdparty/Fitline/LFLineFitter.cpp -o CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.s

CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.requires:
.PHONY : CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.requires

CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.provides: CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.requires
	$(MAKE) -f CMakeFiles/LineFitLib.dir/build.make CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.provides.build
.PHONY : CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.provides

CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.provides.build: CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o

CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o: CMakeFiles/LineFitLib.dir/flags.make
CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o: LFLineSegment.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tox/git/object_tracking_2D/3rdparty/Fitline/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o -c /home/tox/git/object_tracking_2D/3rdparty/Fitline/LFLineSegment.cpp

CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tox/git/object_tracking_2D/3rdparty/Fitline/LFLineSegment.cpp > CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.i

CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tox/git/object_tracking_2D/3rdparty/Fitline/LFLineSegment.cpp -o CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.s

CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.requires:
.PHONY : CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.requires

CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.provides: CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.requires
	$(MAKE) -f CMakeFiles/LineFitLib.dir/build.make CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.provides.build
.PHONY : CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.provides

CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.provides.build: CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o

# Object files for target LineFitLib
LineFitLib_OBJECTS = \
"CMakeFiles/LineFitLib.dir/fitline.cpp.o" \
"CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o" \
"CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o"

# External object files for target LineFitLib
LineFitLib_EXTERNAL_OBJECTS =

libLineFitLib.a: CMakeFiles/LineFitLib.dir/fitline.cpp.o
libLineFitLib.a: CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o
libLineFitLib.a: CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o
libLineFitLib.a: CMakeFiles/LineFitLib.dir/build.make
libLineFitLib.a: CMakeFiles/LineFitLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libLineFitLib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/LineFitLib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LineFitLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LineFitLib.dir/build: libLineFitLib.a
.PHONY : CMakeFiles/LineFitLib.dir/build

CMakeFiles/LineFitLib.dir/requires: CMakeFiles/LineFitLib.dir/fitline.cpp.o.requires
CMakeFiles/LineFitLib.dir/requires: CMakeFiles/LineFitLib.dir/LFLineFitter.cpp.o.requires
CMakeFiles/LineFitLib.dir/requires: CMakeFiles/LineFitLib.dir/LFLineSegment.cpp.o.requires
.PHONY : CMakeFiles/LineFitLib.dir/requires

CMakeFiles/LineFitLib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LineFitLib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LineFitLib.dir/clean

CMakeFiles/LineFitLib.dir/depend:
	cd /home/tox/git/object_tracking_2D/3rdparty/Fitline && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tox/git/object_tracking_2D/3rdparty/Fitline /home/tox/git/object_tracking_2D/3rdparty/Fitline /home/tox/git/object_tracking_2D/3rdparty/Fitline /home/tox/git/object_tracking_2D/3rdparty/Fitline /home/tox/git/object_tracking_2D/3rdparty/Fitline/CMakeFiles/LineFitLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LineFitLib.dir/depend


# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /snap/clion/137/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/137/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ida/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ida/argos3-examples/cmake-build-debug

# Include any dependencies generated for this target.
include controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/depend.make

# Include the progress variables for this target.
include controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/progress.make

# Include the compile flags for this target's objects.
include controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/flags.make

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.o: controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/flags.make
controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.o: controllers/footbot_synchronization/footbot_synchronization_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ida/argos3-examples/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.o"
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.o -c /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization/footbot_synchronization_autogen/mocs_compilation.cpp

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.i"
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization/footbot_synchronization_autogen/mocs_compilation.cpp > CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.i

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.s"
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization/footbot_synchronization_autogen/mocs_compilation.cpp -o CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.s

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.o: controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/flags.make
controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.o: ../controllers/footbot_synchronization/footbot_synchronization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ida/argos3-examples/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.o"
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.o -c /home/ida/argos3-examples/controllers/footbot_synchronization/footbot_synchronization.cpp

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.i"
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ida/argos3-examples/controllers/footbot_synchronization/footbot_synchronization.cpp > CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.i

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.s"
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ida/argos3-examples/controllers/footbot_synchronization/footbot_synchronization.cpp -o CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.s

# Object files for target footbot_synchronization
footbot_synchronization_OBJECTS = \
"CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.o"

# External object files for target footbot_synchronization
footbot_synchronization_EXTERNAL_OBJECTS =

controllers/footbot_synchronization/libfootbot_synchronization.so: controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization_autogen/mocs_compilation.cpp.o
controllers/footbot_synchronization/libfootbot_synchronization.so: controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/footbot_synchronization.cpp.o
controllers/footbot_synchronization/libfootbot_synchronization.so: controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/build.make
controllers/footbot_synchronization/libfootbot_synchronization.so: controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ida/argos3-examples/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module libfootbot_synchronization.so"
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/footbot_synchronization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/build: controllers/footbot_synchronization/libfootbot_synchronization.so

.PHONY : controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/build

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/clean:
	cd /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization && $(CMAKE_COMMAND) -P CMakeFiles/footbot_synchronization.dir/cmake_clean.cmake
.PHONY : controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/clean

controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/depend:
	cd /home/ida/argos3-examples/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ida/argos3-examples /home/ida/argos3-examples/controllers/footbot_synchronization /home/ida/argos3-examples/cmake-build-debug /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization /home/ida/argos3-examples/cmake-build-debug/controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/footbot_synchronization/CMakeFiles/footbot_synchronization.dir/depend


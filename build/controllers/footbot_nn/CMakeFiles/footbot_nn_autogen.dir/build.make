# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ida/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ida/argos3-examples/build

# Utility rule file for footbot_nn_autogen.

# Include the progress variables for this target.
include controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/progress.make

controllers/footbot_nn/CMakeFiles/footbot_nn_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ida/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target footbot_nn"
	cd /home/ida/argos3-examples/build/controllers/footbot_nn && /usr/bin/cmake -E cmake_autogen /home/ida/argos3-examples/build/controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/AutogenInfo.json Debug

footbot_nn_autogen: controllers/footbot_nn/CMakeFiles/footbot_nn_autogen
footbot_nn_autogen: controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/build.make

.PHONY : footbot_nn_autogen

# Rule to build all files generated by this target.
controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/build: footbot_nn_autogen

.PHONY : controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/build

controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/clean:
	cd /home/ida/argos3-examples/build/controllers/footbot_nn && $(CMAKE_COMMAND) -P CMakeFiles/footbot_nn_autogen.dir/cmake_clean.cmake
.PHONY : controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/clean

controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/depend:
	cd /home/ida/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ida/argos3-examples /home/ida/argos3-examples/controllers/footbot_nn /home/ida/argos3-examples/build /home/ida/argos3-examples/build/controllers/footbot_nn /home/ida/argos3-examples/build/controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/footbot_nn/CMakeFiles/footbot_nn_autogen.dir/depend


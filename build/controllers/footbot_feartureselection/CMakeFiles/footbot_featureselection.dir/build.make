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

# Include any dependencies generated for this target.
include controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/depend.make

# Include the progress variables for this target.
include controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/progress.make

# Include the compile flags for this target's objects.
include controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/flags.make

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.o: controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/flags.make
controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.o: controllers/footbot_feartureselection/footbot_featureselection_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ida/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.o"
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.o -c /home/ida/argos3-examples/build/controllers/footbot_feartureselection/footbot_featureselection_autogen/mocs_compilation.cpp

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.i"
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ida/argos3-examples/build/controllers/footbot_feartureselection/footbot_featureselection_autogen/mocs_compilation.cpp > CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.i

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.s"
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ida/argos3-examples/build/controllers/footbot_feartureselection/footbot_featureselection_autogen/mocs_compilation.cpp -o CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.s

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.o: controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/flags.make
controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.o: ../controllers/footbot_feartureselection/footbot_featureselection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ida/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.o"
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.o -c /home/ida/argos3-examples/controllers/footbot_feartureselection/footbot_featureselection.cpp

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.i"
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ida/argos3-examples/controllers/footbot_feartureselection/footbot_featureselection.cpp > CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.i

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.s"
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ida/argos3-examples/controllers/footbot_feartureselection/footbot_featureselection.cpp -o CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.s

# Object files for target footbot_featureselection
footbot_featureselection_OBJECTS = \
"CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.o"

# External object files for target footbot_featureselection
footbot_featureselection_EXTERNAL_OBJECTS =

controllers/footbot_feartureselection/libfootbot_featureselection.so: controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection_autogen/mocs_compilation.cpp.o
controllers/footbot_feartureselection/libfootbot_featureselection.so: controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/footbot_featureselection.cpp.o
controllers/footbot_feartureselection/libfootbot_featureselection.so: controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/build.make
controllers/footbot_feartureselection/libfootbot_featureselection.so: controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ida/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libfootbot_featureselection.so"
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/footbot_featureselection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/build: controllers/footbot_feartureselection/libfootbot_featureselection.so

.PHONY : controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/build

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/clean:
	cd /home/ida/argos3-examples/build/controllers/footbot_feartureselection && $(CMAKE_COMMAND) -P CMakeFiles/footbot_featureselection.dir/cmake_clean.cmake
.PHONY : controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/clean

controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/depend:
	cd /home/ida/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ida/argos3-examples /home/ida/argos3-examples/controllers/footbot_feartureselection /home/ida/argos3-examples/build /home/ida/argos3-examples/build/controllers/footbot_feartureselection /home/ida/argos3-examples/build/controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/footbot_feartureselection/CMakeFiles/footbot_featureselection.dir/depend


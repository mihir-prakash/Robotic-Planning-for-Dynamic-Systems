# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/mihirprakash/Desktop/project4_src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/mihirprakash/Desktop/project4_src/build

# Include any dependencies generated for this target.
include CMakeFiles/car_project.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/car_project.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/car_project.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/car_project.dir/flags.make

CMakeFiles/car_project.dir/codegen:
.PHONY : CMakeFiles/car_project.dir/codegen

CMakeFiles/car_project.dir/src/Project4Car.cpp.o: CMakeFiles/car_project.dir/flags.make
CMakeFiles/car_project.dir/src/Project4Car.cpp.o: /Users/mihirprakash/Desktop/project4_src/src/Project4Car.cpp
CMakeFiles/car_project.dir/src/Project4Car.cpp.o: CMakeFiles/car_project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/mihirprakash/Desktop/project4_src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/car_project.dir/src/Project4Car.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/car_project.dir/src/Project4Car.cpp.o -MF CMakeFiles/car_project.dir/src/Project4Car.cpp.o.d -o CMakeFiles/car_project.dir/src/Project4Car.cpp.o -c /Users/mihirprakash/Desktop/project4_src/src/Project4Car.cpp

CMakeFiles/car_project.dir/src/Project4Car.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/car_project.dir/src/Project4Car.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mihirprakash/Desktop/project4_src/src/Project4Car.cpp > CMakeFiles/car_project.dir/src/Project4Car.cpp.i

CMakeFiles/car_project.dir/src/Project4Car.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/car_project.dir/src/Project4Car.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mihirprakash/Desktop/project4_src/src/Project4Car.cpp -o CMakeFiles/car_project.dir/src/Project4Car.cpp.s

CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o: CMakeFiles/car_project.dir/flags.make
CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o: /Users/mihirprakash/Desktop/project4_src/src/CollisionChecking.cpp
CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o: CMakeFiles/car_project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/mihirprakash/Desktop/project4_src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o -MF CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o.d -o CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o -c /Users/mihirprakash/Desktop/project4_src/src/CollisionChecking.cpp

CMakeFiles/car_project.dir/src/CollisionChecking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/car_project.dir/src/CollisionChecking.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mihirprakash/Desktop/project4_src/src/CollisionChecking.cpp > CMakeFiles/car_project.dir/src/CollisionChecking.cpp.i

CMakeFiles/car_project.dir/src/CollisionChecking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/car_project.dir/src/CollisionChecking.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mihirprakash/Desktop/project4_src/src/CollisionChecking.cpp -o CMakeFiles/car_project.dir/src/CollisionChecking.cpp.s

CMakeFiles/car_project.dir/src/RG-RRT.cpp.o: CMakeFiles/car_project.dir/flags.make
CMakeFiles/car_project.dir/src/RG-RRT.cpp.o: /Users/mihirprakash/Desktop/project4_src/src/RG-RRT.cpp
CMakeFiles/car_project.dir/src/RG-RRT.cpp.o: CMakeFiles/car_project.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/mihirprakash/Desktop/project4_src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/car_project.dir/src/RG-RRT.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/car_project.dir/src/RG-RRT.cpp.o -MF CMakeFiles/car_project.dir/src/RG-RRT.cpp.o.d -o CMakeFiles/car_project.dir/src/RG-RRT.cpp.o -c /Users/mihirprakash/Desktop/project4_src/src/RG-RRT.cpp

CMakeFiles/car_project.dir/src/RG-RRT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/car_project.dir/src/RG-RRT.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mihirprakash/Desktop/project4_src/src/RG-RRT.cpp > CMakeFiles/car_project.dir/src/RG-RRT.cpp.i

CMakeFiles/car_project.dir/src/RG-RRT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/car_project.dir/src/RG-RRT.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mihirprakash/Desktop/project4_src/src/RG-RRT.cpp -o CMakeFiles/car_project.dir/src/RG-RRT.cpp.s

# Object files for target car_project
car_project_OBJECTS = \
"CMakeFiles/car_project.dir/src/Project4Car.cpp.o" \
"CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o" \
"CMakeFiles/car_project.dir/src/RG-RRT.cpp.o"

# External object files for target car_project
car_project_EXTERNAL_OBJECTS =

car_project: CMakeFiles/car_project.dir/src/Project4Car.cpp.o
car_project: CMakeFiles/car_project.dir/src/CollisionChecking.cpp.o
car_project: CMakeFiles/car_project.dir/src/RG-RRT.cpp.o
car_project: CMakeFiles/car_project.dir/build.make
car_project: CMakeFiles/car_project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/mihirprakash/Desktop/project4_src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable car_project"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/car_project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/car_project.dir/build: car_project
.PHONY : CMakeFiles/car_project.dir/build

CMakeFiles/car_project.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/car_project.dir/cmake_clean.cmake
.PHONY : CMakeFiles/car_project.dir/clean

CMakeFiles/car_project.dir/depend:
	cd /Users/mihirprakash/Desktop/project4_src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/mihirprakash/Desktop/project4_src /Users/mihirprakash/Desktop/project4_src /Users/mihirprakash/Desktop/project4_src/build /Users/mihirprakash/Desktop/project4_src/build /Users/mihirprakash/Desktop/project4_src/build/CMakeFiles/car_project.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/car_project.dir/depend

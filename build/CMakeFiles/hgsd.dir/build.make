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
CMAKE_SOURCE_DIR = /mnt/c/Users/glenn/Desktop/thesis/HGSD

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/glenn/Desktop/thesis/HGSD/build

# Include any dependencies generated for this target.
include CMakeFiles/hgsd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hgsd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hgsd.dir/flags.make

CMakeFiles/hgsd.dir/src/Drone.cpp.o: CMakeFiles/hgsd.dir/flags.make
CMakeFiles/hgsd.dir/src/Drone.cpp.o: ../src/Drone.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hgsd.dir/src/Drone.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hgsd.dir/src/Drone.cpp.o -c /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Drone.cpp

CMakeFiles/hgsd.dir/src/Drone.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hgsd.dir/src/Drone.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Drone.cpp > CMakeFiles/hgsd.dir/src/Drone.cpp.i

CMakeFiles/hgsd.dir/src/Drone.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hgsd.dir/src/Drone.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Drone.cpp -o CMakeFiles/hgsd.dir/src/Drone.cpp.s

CMakeFiles/hgsd.dir/src/DVRPD.cpp.o: CMakeFiles/hgsd.dir/flags.make
CMakeFiles/hgsd.dir/src/DVRPD.cpp.o: ../src/DVRPD.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/hgsd.dir/src/DVRPD.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hgsd.dir/src/DVRPD.cpp.o -c /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/DVRPD.cpp

CMakeFiles/hgsd.dir/src/DVRPD.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hgsd.dir/src/DVRPD.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/DVRPD.cpp > CMakeFiles/hgsd.dir/src/DVRPD.cpp.i

CMakeFiles/hgsd.dir/src/DVRPD.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hgsd.dir/src/DVRPD.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/DVRPD.cpp -o CMakeFiles/hgsd.dir/src/DVRPD.cpp.s

CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.o: CMakeFiles/hgsd.dir/flags.make
CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.o: ../src/GeneticAlgorithm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.o -c /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/GeneticAlgorithm.cpp

CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/GeneticAlgorithm.cpp > CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.i

CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/GeneticAlgorithm.cpp -o CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.s

CMakeFiles/hgsd.dir/src/Vehicle.cpp.o: CMakeFiles/hgsd.dir/flags.make
CMakeFiles/hgsd.dir/src/Vehicle.cpp.o: ../src/Vehicle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/hgsd.dir/src/Vehicle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hgsd.dir/src/Vehicle.cpp.o -c /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Vehicle.cpp

CMakeFiles/hgsd.dir/src/Vehicle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hgsd.dir/src/Vehicle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Vehicle.cpp > CMakeFiles/hgsd.dir/src/Vehicle.cpp.i

CMakeFiles/hgsd.dir/src/Vehicle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hgsd.dir/src/Vehicle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Vehicle.cpp -o CMakeFiles/hgsd.dir/src/Vehicle.cpp.s

CMakeFiles/hgsd.dir/src/Routes.cpp.o: CMakeFiles/hgsd.dir/flags.make
CMakeFiles/hgsd.dir/src/Routes.cpp.o: ../src/Routes.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/hgsd.dir/src/Routes.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hgsd.dir/src/Routes.cpp.o -c /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Routes.cpp

CMakeFiles/hgsd.dir/src/Routes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hgsd.dir/src/Routes.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Routes.cpp > CMakeFiles/hgsd.dir/src/Routes.cpp.i

CMakeFiles/hgsd.dir/src/Routes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hgsd.dir/src/Routes.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Routes.cpp -o CMakeFiles/hgsd.dir/src/Routes.cpp.s

CMakeFiles/hgsd.dir/src/Location.cpp.o: CMakeFiles/hgsd.dir/flags.make
CMakeFiles/hgsd.dir/src/Location.cpp.o: ../src/Location.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/hgsd.dir/src/Location.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hgsd.dir/src/Location.cpp.o -c /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Location.cpp

CMakeFiles/hgsd.dir/src/Location.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hgsd.dir/src/Location.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Location.cpp > CMakeFiles/hgsd.dir/src/Location.cpp.i

CMakeFiles/hgsd.dir/src/Location.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hgsd.dir/src/Location.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Location.cpp -o CMakeFiles/hgsd.dir/src/Location.cpp.s

CMakeFiles/hgsd.dir/src/Tests.cpp.o: CMakeFiles/hgsd.dir/flags.make
CMakeFiles/hgsd.dir/src/Tests.cpp.o: ../src/Tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/hgsd.dir/src/Tests.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hgsd.dir/src/Tests.cpp.o -c /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Tests.cpp

CMakeFiles/hgsd.dir/src/Tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hgsd.dir/src/Tests.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Tests.cpp > CMakeFiles/hgsd.dir/src/Tests.cpp.i

CMakeFiles/hgsd.dir/src/Tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hgsd.dir/src/Tests.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/glenn/Desktop/thesis/HGSD/src/Tests.cpp -o CMakeFiles/hgsd.dir/src/Tests.cpp.s

# Object files for target hgsd
hgsd_OBJECTS = \
"CMakeFiles/hgsd.dir/src/Drone.cpp.o" \
"CMakeFiles/hgsd.dir/src/DVRPD.cpp.o" \
"CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.o" \
"CMakeFiles/hgsd.dir/src/Vehicle.cpp.o" \
"CMakeFiles/hgsd.dir/src/Routes.cpp.o" \
"CMakeFiles/hgsd.dir/src/Location.cpp.o" \
"CMakeFiles/hgsd.dir/src/Tests.cpp.o"

# External object files for target hgsd
hgsd_EXTERNAL_OBJECTS =

hgsd: CMakeFiles/hgsd.dir/src/Drone.cpp.o
hgsd: CMakeFiles/hgsd.dir/src/DVRPD.cpp.o
hgsd: CMakeFiles/hgsd.dir/src/GeneticAlgorithm.cpp.o
hgsd: CMakeFiles/hgsd.dir/src/Vehicle.cpp.o
hgsd: CMakeFiles/hgsd.dir/src/Routes.cpp.o
hgsd: CMakeFiles/hgsd.dir/src/Location.cpp.o
hgsd: CMakeFiles/hgsd.dir/src/Tests.cpp.o
hgsd: CMakeFiles/hgsd.dir/build.make
hgsd: CMakeFiles/hgsd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable hgsd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hgsd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hgsd.dir/build: hgsd

.PHONY : CMakeFiles/hgsd.dir/build

CMakeFiles/hgsd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hgsd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hgsd.dir/clean

CMakeFiles/hgsd.dir/depend:
	cd /mnt/c/Users/glenn/Desktop/thesis/HGSD/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/glenn/Desktop/thesis/HGSD /mnt/c/Users/glenn/Desktop/thesis/HGSD /mnt/c/Users/glenn/Desktop/thesis/HGSD/build /mnt/c/Users/glenn/Desktop/thesis/HGSD/build /mnt/c/Users/glenn/Desktop/thesis/HGSD/build/CMakeFiles/hgsd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hgsd.dir/depend


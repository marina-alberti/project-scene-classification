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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/marina/APIs_Scene_Structure_from_DB_v3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marina/APIs_Scene_Structure_from_DB_v3/src

# Include any dependencies generated for this target.
include CMakeFiles/DatabaseInformation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DatabaseInformation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DatabaseInformation.dir/flags.make

CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o: CMakeFiles/DatabaseInformation.dir/flags.make
CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o: DatabaseInformation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marina/APIs_Scene_Structure_from_DB_v3/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o -c /home/marina/APIs_Scene_Structure_from_DB_v3/src/DatabaseInformation.cpp

CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marina/APIs_Scene_Structure_from_DB_v3/src/DatabaseInformation.cpp > CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.i

CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marina/APIs_Scene_Structure_from_DB_v3/src/DatabaseInformation.cpp -o CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.s

CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.requires:
.PHONY : CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.requires

CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.provides: CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.requires
	$(MAKE) -f CMakeFiles/DatabaseInformation.dir/build.make CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.provides.build
.PHONY : CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.provides

CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.provides.build: CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o

# Object files for target DatabaseInformation
DatabaseInformation_OBJECTS = \
"CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o"

# External object files for target DatabaseInformation
DatabaseInformation_EXTERNAL_OBJECTS =

libDatabaseInformation.a: CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o
libDatabaseInformation.a: CMakeFiles/DatabaseInformation.dir/build.make
libDatabaseInformation.a: CMakeFiles/DatabaseInformation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libDatabaseInformation.a"
	$(CMAKE_COMMAND) -P CMakeFiles/DatabaseInformation.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DatabaseInformation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DatabaseInformation.dir/build: libDatabaseInformation.a
.PHONY : CMakeFiles/DatabaseInformation.dir/build

CMakeFiles/DatabaseInformation.dir/requires: CMakeFiles/DatabaseInformation.dir/DatabaseInformation.cpp.o.requires
.PHONY : CMakeFiles/DatabaseInformation.dir/requires

CMakeFiles/DatabaseInformation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DatabaseInformation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DatabaseInformation.dir/clean

CMakeFiles/DatabaseInformation.dir/depend:
	cd /home/marina/APIs_Scene_Structure_from_DB_v3/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marina/APIs_Scene_Structure_from_DB_v3 /home/marina/APIs_Scene_Structure_from_DB_v3 /home/marina/APIs_Scene_Structure_from_DB_v3/src /home/marina/APIs_Scene_Structure_from_DB_v3/src /home/marina/APIs_Scene_Structure_from_DB_v3/src/CMakeFiles/DatabaseInformation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DatabaseInformation.dir/depend

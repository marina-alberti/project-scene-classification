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
include CMakeFiles/FeatureInformation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FeatureInformation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FeatureInformation.dir/flags.make

CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o: CMakeFiles/FeatureInformation.dir/flags.make
CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o: FeatureInformation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marina/APIs_Scene_Structure_from_DB_v3/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o -c /home/marina/APIs_Scene_Structure_from_DB_v3/src/FeatureInformation.cpp

CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marina/APIs_Scene_Structure_from_DB_v3/src/FeatureInformation.cpp > CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.i

CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marina/APIs_Scene_Structure_from_DB_v3/src/FeatureInformation.cpp -o CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.s

CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.requires:
.PHONY : CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.requires

CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.provides: CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.requires
	$(MAKE) -f CMakeFiles/FeatureInformation.dir/build.make CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.provides.build
.PHONY : CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.provides

CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.provides.build: CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o

# Object files for target FeatureInformation
FeatureInformation_OBJECTS = \
"CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o"

# External object files for target FeatureInformation
FeatureInformation_EXTERNAL_OBJECTS =

libFeatureInformation.a: CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o
libFeatureInformation.a: CMakeFiles/FeatureInformation.dir/build.make
libFeatureInformation.a: CMakeFiles/FeatureInformation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libFeatureInformation.a"
	$(CMAKE_COMMAND) -P CMakeFiles/FeatureInformation.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FeatureInformation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FeatureInformation.dir/build: libFeatureInformation.a
.PHONY : CMakeFiles/FeatureInformation.dir/build

CMakeFiles/FeatureInformation.dir/requires: CMakeFiles/FeatureInformation.dir/FeatureInformation.cpp.o.requires
.PHONY : CMakeFiles/FeatureInformation.dir/requires

CMakeFiles/FeatureInformation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FeatureInformation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FeatureInformation.dir/clean

CMakeFiles/FeatureInformation.dir/depend:
	cd /home/marina/APIs_Scene_Structure_from_DB_v3/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marina/APIs_Scene_Structure_from_DB_v3 /home/marina/APIs_Scene_Structure_from_DB_v3 /home/marina/APIs_Scene_Structure_from_DB_v3/src /home/marina/APIs_Scene_Structure_from_DB_v3/src /home/marina/APIs_Scene_Structure_from_DB_v3/src/CMakeFiles/FeatureInformation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FeatureInformation.dir/depend


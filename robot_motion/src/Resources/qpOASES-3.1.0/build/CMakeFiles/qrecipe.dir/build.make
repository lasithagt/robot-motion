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
CMAKE_SOURCE_DIR = /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/build

# Include any dependencies generated for this target.
include CMakeFiles/qrecipe.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/qrecipe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/qrecipe.dir/flags.make

CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o: CMakeFiles/qrecipe.dir/flags.make
CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o: ../examples/qrecipe.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o -c /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/examples/qrecipe.cpp

CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/examples/qrecipe.cpp > CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.i

CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/examples/qrecipe.cpp -o CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.s

CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.requires:

.PHONY : CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.requires

CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.provides: CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.requires
	$(MAKE) -f CMakeFiles/qrecipe.dir/build.make CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.provides.build
.PHONY : CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.provides

CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.provides.build: CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o


# Object files for target qrecipe
qrecipe_OBJECTS = \
"CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o"

# External object files for target qrecipe
qrecipe_EXTERNAL_OBJECTS =

bin/qrecipe: CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o
bin/qrecipe: CMakeFiles/qrecipe.dir/build.make
bin/qrecipe: libs/libqpOASES.a
bin/qrecipe: CMakeFiles/qrecipe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/qrecipe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qrecipe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/qrecipe.dir/build: bin/qrecipe

.PHONY : CMakeFiles/qrecipe.dir/build

CMakeFiles/qrecipe.dir/requires: CMakeFiles/qrecipe.dir/examples/qrecipe.cpp.o.requires

.PHONY : CMakeFiles/qrecipe.dir/requires

CMakeFiles/qrecipe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/qrecipe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/qrecipe.dir/clean

CMakeFiles/qrecipe.dir/depend:
	cd /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0 /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0 /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/build /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/build /home/lasitha/Documents/pkgs_util/Resources/qpOASES-3.1.0/build/CMakeFiles/qrecipe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/qrecipe.dir/depend


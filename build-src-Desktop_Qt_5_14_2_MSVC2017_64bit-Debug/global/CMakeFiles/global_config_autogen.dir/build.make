# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles JOM" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

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

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = E:\_Graduate_Design\project\Registration\src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug

# Utility rule file for global_config_autogen.

# Include any custom commands dependencies for this target.
include global\CMakeFiles\global_config_autogen.dir\compiler_depend.make

# Include the progress variables for this target.
include global\CMakeFiles\global_config_autogen.dir\progress.make

global\CMakeFiles\global_config_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target global_config"
	cd E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug\global
	echo >nul && "C:\Program Files\CMake\bin\cmake.exe" -E cmake_autogen E:/_Graduate_Design/project/Registration/build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug/global/CMakeFiles/global_config_autogen.dir/AutogenInfo.json Debug
	cd E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug

global_config_autogen: global\CMakeFiles\global_config_autogen
global_config_autogen: global\CMakeFiles\global_config_autogen.dir\build.make
.PHONY : global_config_autogen

# Rule to build all files generated by this target.
global\CMakeFiles\global_config_autogen.dir\build: global_config_autogen
.PHONY : global\CMakeFiles\global_config_autogen.dir\build

global\CMakeFiles\global_config_autogen.dir\clean:
	cd E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug\global
	$(CMAKE_COMMAND) -P CMakeFiles\global_config_autogen.dir\cmake_clean.cmake
	cd E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug
.PHONY : global\CMakeFiles\global_config_autogen.dir\clean

global\CMakeFiles\global_config_autogen.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles JOM" E:\_Graduate_Design\project\Registration\src E:\_Graduate_Design\project\Registration\src\global E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug\global E:\_Graduate_Design\project\Registration\build-src-Desktop_Qt_5_14_2_MSVC2017_64bit-Debug\global\CMakeFiles\global_config_autogen.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : global\CMakeFiles\global_config_autogen.dir\depend


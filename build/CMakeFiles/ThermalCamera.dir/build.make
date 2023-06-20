# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/electronica/Desktop/Sistema_Alerta_de_Temperatura

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build

# Include any dependencies generated for this target.
include CMakeFiles/ThermalCamera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ThermalCamera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ThermalCamera.dir/flags.make

CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.o: CMakeFiles/ThermalCamera.dir/flags.make
CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.o: ../src/ThermalCamera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.o -c /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/src/ThermalCamera.cpp

CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/src/ThermalCamera.cpp > CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.i

CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/src/ThermalCamera.cpp -o CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.s

CMakeFiles/ThermalCamera.dir/src/main.cpp.o: CMakeFiles/ThermalCamera.dir/flags.make
CMakeFiles/ThermalCamera.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ThermalCamera.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ThermalCamera.dir/src/main.cpp.o -c /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/src/main.cpp

CMakeFiles/ThermalCamera.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThermalCamera.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/src/main.cpp > CMakeFiles/ThermalCamera.dir/src/main.cpp.i

CMakeFiles/ThermalCamera.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThermalCamera.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/src/main.cpp -o CMakeFiles/ThermalCamera.dir/src/main.cpp.s

# Object files for target ThermalCamera
ThermalCamera_OBJECTS = \
"CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.o" \
"CMakeFiles/ThermalCamera.dir/src/main.cpp.o"

# External object files for target ThermalCamera
ThermalCamera_EXTERNAL_OBJECTS =

ThermalCamera: CMakeFiles/ThermalCamera.dir/src/ThermalCamera.cpp.o
ThermalCamera: CMakeFiles/ThermalCamera.dir/src/main.cpp.o
ThermalCamera: CMakeFiles/ThermalCamera.dir/build.make
ThermalCamera: libmlx90640_api.a
ThermalCamera: /usr/local/lib/arm-linux-gnueabihf/libTgBot.a
ThermalCamera: /usr/lib/arm-linux-gnueabihf/libssl.so
ThermalCamera: /usr/lib/arm-linux-gnueabihf/libcrypto.so
ThermalCamera: /usr/lib/arm-linux-gnueabihf/libSDL2_ttf.so
ThermalCamera: /usr/lib/arm-linux-gnueabihf/libSDL2.so
ThermalCamera: CMakeFiles/ThermalCamera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ThermalCamera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ThermalCamera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ThermalCamera.dir/build: ThermalCamera

.PHONY : CMakeFiles/ThermalCamera.dir/build

CMakeFiles/ThermalCamera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ThermalCamera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ThermalCamera.dir/clean

CMakeFiles/ThermalCamera.dir/depend:
	cd /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/electronica/Desktop/Sistema_Alerta_de_Temperatura /home/electronica/Desktop/Sistema_Alerta_de_Temperatura /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build /home/electronica/Desktop/Sistema_Alerta_de_Temperatura/build/CMakeFiles/ThermalCamera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ThermalCamera.dir/depend

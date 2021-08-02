# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rosaria)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation roscpp nav_msgs geometry_msgs sensor_msgs tf
  dynamic_reconfigure)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
  FILES
  BumperState.msg
)
#uncomment if you have defined services
#add_service_files(
#  FILES
#  # TODO: List your msg files here
#)

generate_dynamic_reconfigure_options(cfg/RosAria.cfg)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

find_package(Aria QUIET)
if(Aria_FOUND)
  message("CMake package for Aria was found, using that.")
  # Aria package for CMake was found
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
  # No Aria package for CMake was found, search ourselves
	# The installation package provided by Adept doesn't follow normal Linux
	# installation locations, but the repackaged Debian package and AriaCoda do.
  # If user set ARIA environment variable, look there, otherwise look in
  # /usr/local/.
  if(DEFINED ENV{ARIA})
    message("ARIA environment variable defined, checking there...")
    set(prefix $ENV{ARIA})
  else()
    set(prefix "/usr/local")
  endif()
  message("Looking for Aria in ${prefix}/Aria and ${prefix}...")

	if(EXISTS "${prefix}/Aria/include/Aria.h")
    message("Found ${prefix}/Aria/include/Aria.h, assuming Adept ARIA package.")
	  add_definitions(-DADEPT_PKG)
	  include_directories(${prefix}/Aria/include)
	  link_directories(${prefix}/Aria/lib)
  else()
    if(EXISTS "${prefix}/include/Aria.h")
      message("Found ${prefix)/include/Aria.h, assuming Adept ARIA source directory.")
      add_definitions(-DADEPT_PKG)
      include_directories(${prefix}/include)
      link_directories(${prefix}/lib)
    else()
      if(EXISTS "${prefix}/include/Aria/Aria.h")
        message("Found ${prefix}/include/Aria/Aria.h, assuming AriaCoda or repackaged ARIA.")
        #add_definitions(-DARIACODA)
        include_directories(${prefix}/include)
        link_directories(${prefix}/lib)
      else()
        message("Aria.h not found in ${prefix}. Continuing with default header and library paths.")
      endif()
    endif()
	endif()
endif()

add_executable(RosAria RosAria.cpp LaserPublisher.cpp)
add_dependencies(RosAria rosaria_gencfg)
add_dependencies(RosAria rosaria_gencpp)

target_link_libraries(RosAria ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)
set_target_properties(RosAria PROPERTIES COMPILE_FLAGS "-fPIC")
#set_target_properties(RosAria PROPERTIES LINK_FLAGS "-Wl")

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS 
    RosAria
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

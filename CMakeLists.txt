#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(disposvel)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )
orocos_component(disposvel src/src_classes/Segment.cpp src/src_classes/Cylinder.cpp src/src_classes/Distance2.cpp src/src_classes/Distance.cpp include/disposvel-component.hpp src/disposvel-component.cpp src/support.cpp)
#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_install_headers( include/disposvel-component.hpp ) # ...you may add multiple header files
orocos_install_headers( include/segment.h )
orocos_install_headers( include/cylinder.h )
orocos_install_headers( include/distance.h )
orocos_install_headers( include/distance2.h )
orocos_install_headers( include/support.h )

orocos_generate_package()

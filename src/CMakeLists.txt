
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libdisposvel-<target>.so
# and installs in the directory lib/orocos/disposvel/
#
# ...you may add multiple source files
#
# You may add multiple orocos_component statements.
orocos_component(disposvel support.cpp src_classes/Segment.cpp src_classes/Cylinder.cpp src_classes/Distance2.cpp  src_classes/Distance.cpp ../include/disposvel-component.hpp disposvel-component.cpp)
#
# Additional headers:
#
# Installs in the include/orocos/disposvel/ directory
#
orocos_install_headers(disposvel-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libdisposvel-plugin-<target>.so
# and installs in the directory lib/orocos/disposvel/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(disposvel-plugin disposvel-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libdisposvel-service-<target>.so
# and installs in the directory lib/orocos/disposvel/plugins/
#
orocos_service(disposvel-service disposvel-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libdisposvel-types-<target>.so
# and installs in the directory lib/orocos/@target@/disposvel/types/
#
# The header will go in include/orocos/disposvel/types/disposvel/disposvel-types.hpp
# So you can #include <disposvel/disposvel-types.hpp>
#
orocos_typegen_headers(include/disposvel/disposvel-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !

#orocos_library(functions src_classes/functions.cpp)
orocos_library(segment src_classes/Segment.cpp)
orocos_library(cylinder src_classes/Cylinder.cpp)
orocos_library(distance src_classes/Distance.cpp)
orocos_library(distance2 src_classes/Distance2.cpp)

orocos_library(support support.cpp)
#target_link_libraries(functions segment)

target_link_libraries(distance segment)
target_link_libraries(cylinder segment)
target_link_libraries(distance2 cylinder)
target_link_libraries(disposvel segment)
target_link_libraries(disposvel cylinder)
target_link_libraries(disposvel distance)
target_link_libraries(disposvel distance2)

target_link_libraries(support segment)
target_link_libraries(support distance)

#target_link_libraries(disposvel functions)
#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.



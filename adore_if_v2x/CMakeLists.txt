# ********************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *   Stephan Lapoehn
# ********************************************************************************
cmake_minimum_required(VERSION 3.13)
project(adore_if_v2x)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set(PACKAGE_VENDOR "DLR TS-KOS-SAI")
set(PACKAGE_DESCRIPTION "Automated Driving Open Research: adore if v2x")
set(PACKAGE_MAINTAINER "DLR TS-KOS-SAI")
set(PACKAGE_MAINTAINER_EMAIL "ts-kos-sai@dlr.de")

include(GNUInstallDirs)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Packing)


#set(CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH};/tmp/v2x_if_ros_msg/build/devel;/tmp/adore_if_ros_msg/build/devel;/tmp/coordinate_conversion/build/install;/tmp/plotlablib/plotlablib/build/install")
list(APPEND CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../adore_if_ros_msg/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../v2x_if_ros_msg/v2x_if_ros_msg/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../coordinate_conversion/build/install")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../plotlablib/plotlablib/build/install")
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    adore_if_ros_msg
    dsrc_v2_dsrc
    denm_v2_denm_pdu_descriptions
    dsrc_v2_mapem_pdu_descriptions
    dsrc_v2_spatem_pdu_descriptions
    dsrc_v2_srem_pdu_descriptions
    cpm_v1_19_cpm_pdu_descriptions
    mcm_dmove_mcm_dmove
    coordinate_conversion
    plotlablib
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES v2x_if_ros
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs nav_msgs adore_if_ros_msg  mcm_dmove_mcm_dmove
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/v2x_if_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/v2x_if_ros_node.cpp)
# target_sources(channel_sim_node
#   PRIVATE
# #    src/...cpp
# )
# add_executable(${PROJECT_NAME}_node src/adore_if_v2x_node.cpp)

add_executable(srem_generator_node)
target_sources(srem_generator_node PRIVATE src/srem_generator_node.cpp)
target_include_directories( srem_generator_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(srem_generator_node PRIVATE ${catkin_LIBRARIES} coordinate_conversion::coordinate_conversion)

add_executable(denm_generator_node)
target_sources(denm_generator_node PRIVATE src/denm_generator_node.cpp)
target_include_directories( denm_generator_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(denm_generator_node PRIVATE ${catkin_LIBRARIES} coordinate_conversion::coordinate_conversion)

add_executable(denm_to_bordertypechangeprofile_node)
target_sources(denm_to_bordertypechangeprofile_node PRIVATE src/denm_to_bordertypechangeprofile_node.cpp)
target_include_directories( denm_to_bordertypechangeprofile_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(denm_to_bordertypechangeprofile_node PRIVATE ${catkin_LIBRARIES} coordinate_conversion::coordinate_conversion)

add_executable(denm_to_checkpoint)
target_sources(denm_to_checkpoint PRIVATE src/denm_to_checkpoint.cpp)
target_include_directories( denm_to_checkpoint PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(denm_to_checkpoint PRIVATE ${catkin_LIBRARIES} coordinate_conversion::coordinate_conversion)

#add_executable(mcm_to_prediction_node)
#target_sources(mcm_to_prediction_node PRIVATE src/mcm_to_prediction_node.cpp)
#target_include_directories( mcm_to_prediction_node PRIVATE include ${catkin_INCLUDE_DIRS})
#target_link_libraries(mcm_to_prediction_node PRIVATE ${catkin_LIBRARIES} )

#add_executable(mcm_to_platoon_node)
#target_sources(mcm_to_platoon_node PRIVATE src/mcm_to_platoon_node.cpp)
#target_include_directories( mcm_to_platoon_node PRIVATE include ${catkin_INCLUDE_DIRS})
#target_link_libraries(mcm_to_platoon_node PRIVATE ${catkin_LIBRARIES} )

#add_executable(setpointrequest_to_mcm_node)
#target_sources(setpointrequest_to_mcm_node PRIVATE src/setpointrequest_to_mcm_node.cpp)
#target_include_directories(setpointrequest_to_mcm_node PRIVATE include ${catkin_INCLUDE_DIRS})
#target_link_libraries(setpointrequest_to_mcm_node PRIVATE ${catkin_LIBRARIES} )

add_executable(v2x_trafficlights_node src/v2x_trafficlights_node.cpp)
target_sources(v2x_trafficlights_node PRIVATE src/v2x_trafficlights_node.cpp)
target_include_directories(v2x_trafficlights_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(v2x_trafficlights_node PRIVATE ${catkin_LIBRARIES} coordinate_conversion::coordinate_conversion)

#add_executable(denm_plotter_node)
#target_sources(denm_plotter_node PRIVATE src/denm_plotter_node.cpp)
#target_include_directories(denm_plotter_node PRIVATE include ${catkin_INCLUDE_DIRS})
#target_link_libraries(denm_plotter_node PRIVATE ${catkin_LIBRARIES} plotlablib)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS
    srem_generator_node
    denm_generator_node
    denm_to_bordertypechangeprofile_node
    denm_to_checkpoint
#    mcm_to_prediction_node
#    mcm_to_platoon_node
#    setpointrequest_to_mcm_node
    v2x_trafficlights_node
#    denm_plotter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)





## Mark cpp header files for installation
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})


include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/LibraryConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

install(EXPORT ${PROJECT_NAME}_Targets
        COMPONENT ${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              # "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
        COMPONENT ${PROJECT_NAME}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/build/install/include DESTINATION .) 


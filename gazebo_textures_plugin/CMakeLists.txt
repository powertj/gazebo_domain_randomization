cmake_minimum_required(VERSION 2.8.3)
project(gazebo_textures_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roslib
  roscpp
  geometry_msgs
  std_srvs
  std_msgs
  gazebo_msgs
  gazebo_ext_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  LIBRARIES
    gazebo_textures_plugin

  CATKIN_DEPENDS
    roslib
    roscpp
    gazebo_ros
    geometry_msgs
    std_srvs
    std_msgs
    gazebo_msgs
    gazebo_ext_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()

## Plugins
add_library(gazebo_textures_plugin src/gazebo_textures_plugin.cpp)
add_dependencies(gazebo_textures_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(gazebo_textures_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_textures_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_textures_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})

# Install Gazebo System Plugins
install(TARGETS gazebo_textures_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

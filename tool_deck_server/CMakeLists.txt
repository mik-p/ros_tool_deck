cmake_minimum_required(VERSION 3.10.2)
project(tool_deck_server)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
roscpp
geometry_msgs
visualization_msgs
tf2
tf2_ros
tf2_geometry_msgs
interactive_markers
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

find_package(yaml-cpp REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
roscpp
geometry_msgs
visualization_msgs
tf2
tf2_ros
tf2_geometry_msgs
interactive_markers
DEPENDS yaml-cpp
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

# tool deck lib
add_library(${PROJECT_NAME}
  src/tool_deck.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  # ${YAML_LIBRARIES}
  yaml-cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# node
add_executable(tool_deck_node
  src/tool_deck_node.cpp
)

target_link_libraries(tool_deck_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_LIBRARIES}
)
add_dependencies(tool_deck_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

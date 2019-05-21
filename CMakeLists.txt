cmake_minimum_required(VERSION 2.8.3)
project(rgb_lights_manager)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    diagnostic_msgs
    smach_msgs
    tue_msgs
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# add_library(library_name
#     src/lib_source_file1.cpp
#     ...
# )
# target_link_libraries(library_name ${catkin_LIBRARIES})

add_executable(rgb_lights_manager 
    src/lights_manager.cpp
)
target_link_libraries(rgb_lights_manager ${catkin_LIBRARIES})
add_dependencies(rgb_lights_manager ${catkin_EXPORTED_TARGETS})


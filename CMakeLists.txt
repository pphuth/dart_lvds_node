cmake_minimum_required(VERSION 2.8.3)
project(zynq_camera)

set(RUNTIME_DEPS
    cv_bridge
    sensor_msgs
    image_transport
    )


find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    ${RUNTIME_DEPS}
    )

# generate the dynamic_reconfigure config file
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${RUNTIME_DEPS}
    DEPENDS OpenCV
    )

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Pylon_INCLUDE_DIRS}
)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/ros_camera_zynq.cpp
    )

## Declare a C++ executable
add_executable(zynq_camera_node src/ros_camera_zynq.cpp)
#add_dependencies(zynq_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries( zynq_camera_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Pylon_LIBRARIES}
    bconctl
    )

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

#install(DIRECTORY include/${PROJECT_NAME}/
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

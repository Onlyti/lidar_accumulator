cmake_minimum_required(VERSION 3.2)
project(lidar_accumulator)

option(USE_SYSTEM_TSLMAP "Use system pre-installed tsl_robin" ON)

add_compile_options(-std=c++17)

set(CATKIN_PACKAGE_DEPENDENCIES
)

find_package(catkin REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
 src/lidar_accumulator.cpp
)

add_dependencies(${PROJECT_NAME} 
    # external_open3d
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")

target_link_libraries(${PROJECT_NAME}
    # INTERFACE Open3D
    PUBLIC ${LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    tsl::robin_map
)
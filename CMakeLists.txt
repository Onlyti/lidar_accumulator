cmake_minimum_required(VERSION 3.2)
project(lidar_accumulator)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
  include
  include/tsl
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
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC ${LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
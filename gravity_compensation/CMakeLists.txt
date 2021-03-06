cmake_minimum_required(VERSION 2.8.3)
project(gravity_compensation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp geometry_msgs sensor_msgs tf tf_conversions eigen_conversions)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread QUIET)

catkin_package(
  DEPENDS EIGEN3
  CATKIN_DEPENDS geometry_msgs sensor_msgs tf tf_conversions eigen_conversions
  INCLUDE_DIRS include 
  LIBRARIES gravity_compensation
)

include_directories(include 
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(gravity_compensation src/gravity_compensation.cpp src/gravity_compensation_params.cpp)
target_link_libraries(gravity_compensation ${catkin_LIBRARIES})

add_executable(gravity_compensation_node src/gravity_compensation_node.cpp)
target_link_libraries(gravity_compensation_node gravity_compensation ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS gravity_compensation gravity_compensation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

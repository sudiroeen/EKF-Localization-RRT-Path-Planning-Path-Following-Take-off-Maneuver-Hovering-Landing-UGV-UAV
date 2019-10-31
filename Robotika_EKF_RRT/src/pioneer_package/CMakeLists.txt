cmake_minimum_required(VERSION 2.8.3)
project(pioneer_package)

INCLUDE(FindPkgConfig)

IF(CMAKE_COMPILER_IS_GNUCC)
SET(CMAKE_CXX_FLAGS
"${CMAKE_CXX_FLAGS} -Wall -std=c++0x -DBUILD_SHARED_LIBS=OFF")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

# definicoes da remoteAPI para CPP
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  common_include
  rrt
  geometry_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pioneer_package
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
   src/pioneer_package.cpp 
   src/pioneer_package_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node  
                        ${catkin_LIBRARIES}
                        ${CMAKE_THREAD_LIBS_INIT}
                        ${OpenCV_LIBS}
                        ${Eigen3_LIBS}
                     )
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION bin)

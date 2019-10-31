cmake_minimum_required(VERSION 2.8.3)
project(common_include)

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
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES common_include
)
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/extApi.h
   include/${PROJECT_NAME}/extApiCustom.h
   include/${PROJECT_NAME}/extApiCustomConst.h
   include/${PROJECT_NAME}/extApiInternal.h
   include/${PROJECT_NAME}/extApiPlatform.h
   include/${PROJECT_NAME}/v_repConst.h
   include/${PROJECT_NAME}/v_repLib.h
   include/${PROJECT_NAME}/v_repTypes.h
   
   src/extApi.c
   src/extApiCustom.c
   src/extApiPlatform.c
#   src/simxCustomCmd.cpp
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

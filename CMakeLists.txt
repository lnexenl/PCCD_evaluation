cmake_minimum_required(VERSION 3.2)
project(PCCD_evaluation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-g -Wall -O3 -fPIC -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 -march=native -fpermissive")
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(catkin REQUIRED COMPONENTS fast_gicp)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(${OCTOMAP_INCLUDE_DIR} ${PCL_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR})
include_directories(include)

add_executable(build_global_pc_ot src/build_global_pc_ot.cpp)
target_link_libraries(build_global_pc_ot ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(build_confusion_mat src/build_confusion_matrix.cc)
target_link_libraries(build_confusion_mat ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} OpenMP::OpenMP_CXX)

#
cmake_minimum_required(VERSION 3.0.2)
project(orbslam3_ros)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

set(ORB_SLAM3_ROOT "$ENV{HOME}/third_party/ORB_SLAM3" CACHE PATH "ORB-SLAM3 source/build root")
find_library(ORB_SLAM3_LIBRARY
  NAMES ORB_SLAM3
  PATHS "${ORB_SLAM3_ROOT}/lib"
  NO_DEFAULT_PATH
)
set(DBOW2_LIBRARY "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
set(G2O_LIBRARY "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so")
if(NOT EXISTS "${ORB_SLAM3_ROOT}/include/System.h")
  message(FATAL_ERROR "ORB_SLAM3_ROOT does not contain include/System.h: ${ORB_SLAM3_ROOT}")
endif()
if(NOT ORB_SLAM3_LIBRARY)
  message(FATAL_ERROR "libORB_SLAM3 was not found under ${ORB_SLAM3_ROOT}/lib; run build.sh first")
endif()
if(NOT EXISTS "${DBOW2_LIBRARY}" OR NOT EXISTS "${G2O_LIBRARY}")
  message(FATAL_ERROR "ORB-SLAM3 bundled DBoW2/g2o libraries are missing; run ORB_SLAM3/build.sh first")
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
)

add_executable(orbslam3_mono_node src/mono_node.cpp)
target_link_libraries(orbslam3_mono_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ORB_SLAM3_LIBRARY}
  ${DBOW2_LIBRARY}
  ${G2O_LIBRARY}
  ${Boost_LIBRARIES}
  OpenSSL::Crypto
)

install(TARGETS orbslam3_mono_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

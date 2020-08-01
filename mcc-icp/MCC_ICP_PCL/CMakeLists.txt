cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Debug)
project(mcc_icp_pcl)

#find_package(Eigen3 REQUIRED)
find_package(PCL 1.9 REQUIRED PATHS /home/ashu/libraries/pcl/install_dir/share/pcl-1.9)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (mcc_icp mcc_icp.cpp)
target_link_libraries (mcc_icp ${PCL_LIBRARIES})


add_executable (transform_pointcloud transform_pointcloud.cpp)
target_link_libraries (transform_pointcloud ${PCL_LIBRARIES})

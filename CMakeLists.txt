cmake_minimum_required (VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)
project (ekf)
 
find_package (Eigen3 3.3 REQUIRED)
 
include_directories(${EIGEN3_INCLUDE_DIR}
                    ${PROJECT_SOURCE_DIR}
                                            )

add_executable (ekf ${CMAKE_SOURCE_DIR}/ekftest.cpp ${CMAKE_SOURCE_DIR}/ekf.cpp)
target_link_libraries (ekf Eigen3::Eigen)

set(CMAKE_OSX_ARCHITECTURES "arm64")
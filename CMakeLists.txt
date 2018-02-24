cmake_minimum_required(VERSION 3.4)
project(slam-learning)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE Debug CACHE STRING "set build type to debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb ")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
Find_Package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")
# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# G2O
find_package(G2O REQUIRED)
# Cholmod
Find_Package(Cholmod REQUIRED)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
        g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
        g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${G2O_LIBS}
        ${CHOLMOD_LIBRARIES})

############### source and test ######################
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/include/ceres)
include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/tools ${PROJECT_SOURCE_DIR}/include/beop
        ${PROJECT_SOURCE_DIR}/include/flags ${PROJECT_SOURCE_DIR}/include/g2o ${PROJECT_SOURCE_DIR}/include/common)
add_subdirectory(src)
add_subdirectory(test)
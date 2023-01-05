cmake_minimum_required(VERSION 3.1)
project(lidar_ouster VERSION 0.0)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
include(lidar_ousterBase)

if (ROCK_TEST_ENABLED)
    enable_testing()
    find_package(Syskit REQUIRED)
    syskit_orogen_tests(test)
endif()

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)


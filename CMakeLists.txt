cmake_minimum_required(VERSION 3.1)
project(ced_detector)

set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Default build type: RelWithDebInfo" FORCE)
endif ()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test_registration src/test_registration.cpp)
target_link_libraries (test_registration ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} teaserpp::teaser_registration)
add_executable (test_keypoint src/test_keypoint.cpp)
target_link_libraries (test_keypoint ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})

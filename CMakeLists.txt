cmake_minimum_required(VERSION 3.10.2)

project(ground_filter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

include(cmake/global_defination.cmake)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             pcl_ros
             tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

link_directories(${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS}
)

catkin_package()

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE APP_SRCS "src/apps/*.cpp")
list(REMOVE_ITEM  ALL_SRCS ${APP_SRCS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED 
            ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME}
                      ${PCL_LIBRARIES}
                      ${JSONCPP_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
               src/apps/ground_filter_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node  ${PROJECT_NAME})
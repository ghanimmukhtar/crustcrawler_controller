cmake_minimum_required(VERSION 2.8)
project(crustcrawler_controllers)

#Libraries
#find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system )
link_directories("/usr/local/lib")

include_directories(/usr/include/eigen3)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(LIBRARY_OUTPUT_PATH lib/${CMAKE_BUILD_TYPE})

include_directories(include)

install(DIRECTORY include/  DESTINATION /usr/local/include/${PROJECT_NAME})

file(GLOB_RECURSE src_files src/* include/*)

add_library(real_controllers ${src_files})
target_link_libraries(real_controllers dynamixel)

install(TARGETS real_controllers
    ARCHIVE DESTINATION /usr/local/lib
    LIBRARY DESTINATION /usr/local/lib)

add_executable(test_real_crustcrawler_arm src/test_real_crustcrawler_arm.cpp)
target_link_libraries(test_real_crustcrawler_arm real_controllers dynamixel)

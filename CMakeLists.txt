cmake_minimum_required(VERSION 2.8)
project(crustcrawler_controllers)

#Libraries
#find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system )
find_package(ODE REQUIRED)
link_directories(/usr/local/lib)

include_directories(/usr/include/eigen3 usr/local/include ${ODE_INCLUDE_DIR})

add_definitions(-DdDOUBLE)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(LIBRARY_OUTPUT_PATH lib/${CMAKE_BUILD_TYPE})
set(OSG_LIBRARIES  osg osgAnimation osgDB osgFX osgGA osgManipulator osgParticle osgPresentation osgQt osgShadow osgSim osgTerrain osgText osgUtil osgViewer osgVolume osgWidget)

include_directories(include)

install(DIRECTORY include/  DESTINATION /usr/local/include/${PROJECT_NAME})

file(GLOB_RECURSE src_files src/* include/*)
file(GLOB_RECURSE simu_files test_tools/simulation/*)

add_library(real_controllers ${src_files})
target_link_libraries(real_controllers dynamixel)

add_library(simu_controllers ${simu_files})
target_link_libraries(simu_controllers ${ODE_LIBRARY} robdyn  ${OSG_LIBRARIES} robdyn_osgvisitor)

install(TARGETS real_controllers
    ARCHIVE DESTINATION /usr/local/lib
    LIBRARY DESTINATION /usr/local/lib)

add_executable(test_real_crustcrawler_arm src/test_real_crustcrawler_arm.cpp)
target_link_libraries(test_real_crustcrawler_arm real_controllers dynamixel)

add_executable(ik_simu_test test_tools/ik_simu_test.cpp)
target_link_libraries(ik_simu_test simu_controllers ${OSG_LIBRARIES} robdyn_osgvisitor real_controllers)

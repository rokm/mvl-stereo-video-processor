cmake_minimum_required(VERSION 2.8)
project(mvl-stereo-video-processor CXX)

include(GNUInstallDirs)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -pthread)
endif()

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED core highgui)
find_package(Qt5Core REQUIRED)
find_package(Boost 1.42 COMPONENTS program_options REQUIRED)

# Stereo pipeline library
pkg_check_modules(STEREO_PIPELINE REQUIRED libmvl_stereo_pipeline)
include_directories(${STEREO_PIPELINE_INCLUDE_DIRS})
link_directories(${STEREO_PIPELINE_LIBRARY_DIRS})

# *** Stereo rectification ***
add_executable(mvl-stereo-video-processor
    main.cpp
)

target_link_libraries(mvl-stereo-video-processor
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${STEREO_PIPELINE_LIBRARIES})
qt5_use_modules(mvl-stereo-video-processor Core Concurrent)

install(TARGETS mvl-stereo-video-processor DESTINATION ${CMAKE_INSTALL_BINDIR})

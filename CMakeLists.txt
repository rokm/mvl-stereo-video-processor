cmake_minimum_required(VERSION 2.8.11)
project(mvl-stereo-video-processor C CXX)

include(GNUInstallDirs)

set(CMAKE_AUTOMOC TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

find_package(OpenCV 3.0.0 REQUIRED core imgcodecs videoio)
find_package(Qt5Core REQUIRED)
find_package(Boost 1.42 COMPONENTS program_options REQUIRED)
add_definitions(-DBOOST_ALL_DYN_LINK) # Disable autolinking on MSVC

find_package(libmvl_stereo_pipeline 2.0.0 REQUIRED)
find_package(libvrms 1.0.0 QUIET)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${libmvl_stereo_pipeline_INCLUDE_DIRS})

if (libvrms_FOUND)
    include_directories(${libvrms_INCLUDE_DIRS})
    add_definitions(-DENABLE_VRMS)
    message(STATUS "Enabling VRMS support")
else ()
    message(STATUS "Disabling VRMS support")
endif ()

# *** Stereo rectification ***
add_executable(mvl-stereo-video-processor
    main.cpp
)

target_link_libraries(mvl-stereo-video-processor opencv_core opencv_imgcodecs opencv_videoio)
target_link_libraries(mvl-stereo-video-processor ${libmvl_stereo_pipeline_LIBRARIES})
target_link_libraries(mvl-stereo-video-processor Qt5::Core)
target_link_libraries(mvl-stereo-video-processor ${Boost_LIBRARIES})
if(libvrms_FOUND)
    target_link_libraries(mvl-stereo-video-processor ${libvrms_LIBRARIES})
endif()

install(TARGETS mvl-stereo-video-processor DESTINATION ${CMAKE_INSTALL_BINDIR})

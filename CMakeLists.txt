cmake_minimum_required(VERSION 2.8.11)
project(mvl-stereo-processor C CXX)

include(GNUInstallDirs)

set(CMAKE_AUTOMOC TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

find_package(OpenCV 3.0.0 REQUIRED core imgcodecs videoio)
find_package(Qt5Core REQUIRED)

find_package(libmvl_stereo_pipeline 2.1.0 REQUIRED)
find_package(libvrms 1.0.0 QUIET)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${libmvl_stereo_pipeline_INCLUDE_DIRS})

if (libvrms_FOUND)
    include_directories(${libvrms_INCLUDE_DIRS})
    add_definitions(-DENABLE_VRMS)
    message(STATUS "Enabling VRMS support")
else ()
    message(STATUS "Disabling VRMS support")
endif ()

# *** Stereo rectification ***
add_executable(mvl-stereo-processor
    debug.h
    debug.cpp
    main.cpp
    processor.h
    processor.cpp
    source.cpp
    source_image.h
    source_image.cpp
    source_video.h
    source_video.cpp
    source_vrms.h
    source_vrms.cpp
    utils.h
    utils.cpp
)

target_link_libraries(mvl-stereo-processor opencv_core opencv_imgcodecs opencv_videoio)
target_link_libraries(mvl-stereo-processor ${libmvl_stereo_pipeline_LIBRARIES})
target_link_libraries(mvl-stereo-processor Qt5::Core)
if(libvrms_FOUND)
    target_link_libraries(mvl-stereo-processor ${libvrms_LIBRARIES})
endif()

install(TARGETS mvl-stereo-processor DESTINATION ${CMAKE_INSTALL_BINDIR})

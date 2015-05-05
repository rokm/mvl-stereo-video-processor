cmake_minimum_required(VERSION 2.8.11)
project(mvl-stereo-video-processor CXX)

include(GNUInstallDirs)

set(CMAKE_AUTOMOC TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11 -pthread)
endif()

find_package(OpenCV 3.0.0 REQUIRED core imgcodecs videoio)
find_package(Qt5Core REQUIRED)
find_package(Boost 1.42 COMPONENTS program_options REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Stereo pipeline library
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    # If available, use pkg-config...
    pkg_check_modules(STEREO_PIPELINE REQUIRED libmvl_stereo_pipeline>=2.0.0)
    pkg_check_modules(VRMS libvrms>=1.0.0)
else()
    # ... otherwise, have user specify the path to toolbox installation
    set(MVL_STEREO_TOOLBOX_DIR "" CACHE PATH "Path to MVL Stereo Toolbox")

    set(STEREO_PIPELINE_INCLUDE_DIRS "${MVL_STEREO_TOOLBOX_DIR}/include/MVL")
    set(STEREO_PIPELINE_LIBRARY_DIRS "${MVL_STEREO_TOOLBOX_DIR}/lib")

    set(STEREO_PIPELINE_LIBRARIES mvl_stereo_pipeline)
endif()

include_directories(${STEREO_PIPELINE_INCLUDE_DIRS})
link_directories(${STEREO_PIPELINE_LIBRARY_DIRS})

if(VRMS_FOUND)
    include_directories(${VRMS_INCLUDE_DIRS})
    link_directories(${VRMS_LIBRARY_DIRS})
    add_definitions(-DENABLE_VRMS)
endif()

# *** Stereo rectification ***
add_executable(mvl-stereo-video-processor
    main.cpp
)

if(NOT MSVC)
    # On Visual Studio compilers Boost headers request automatic
    # linking to corresponding libraries...
    target_link_libraries(mvl-stereo-video-processor ${Boost_LIBRARIES})
endif()
target_link_libraries(mvl-stereo-video-processor opencv_core opencv_imgcodecs opencv_videoio)
target_link_libraries(mvl-stereo-video-processor ${STEREO_PIPELINE_LIBRARIES})
target_link_libraries(mvl-stereo-video-processor Qt5::Core)
if(VRMS_FOUND)
    target_link_libraries(mvl-stereo-video-processor ${VRMS_LIBRARIES})
endif()

install(TARGETS mvl-stereo-video-processor DESTINATION ${CMAKE_INSTALL_BINDIR})

cmake_minimum_required(VERSION 3.22)
project(Distributed_SLAM)

set(CMAKE_CXX_STANDARD 17)

add_executable(Distributed_SLAM main.cpp)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif ()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

include_directories(

        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
        ${PROJECT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(ORB_SLAM3 SHARED
        src/Settings.cc
        src/Frame.cc
        src/Config.cc
        src/GeometricTools.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/MLPnPsolver.cpp
        src/OptimizableTypes.cpp
        src/Optimizer.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/Sim3Solver.cc
        src/System.cc
        src/Tracking.cc
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/Atlas/Atlas.cc
        src/Atlas/KeyFrame.cc
        src/Atlas/KeyFrameDatabase.cc
        src/Atlas/MapPoint.cc
        src/Atlas/Map.cc
        src/Types/ImuTypes.cc
        src/Types/G2oTypes.cc
        src/Types/Converter.cc
        src/Visualise/Viewer.cc
        src/Visualise/TwoViewReconstruction.cc
        src/Visualise/MapDrawer.cc
        src/Visualise/FrameDrawer.cc
        include/Settings.h
        include/Frame.h
        include/Config.h
        include/LocalMapping.h
        include/MLPnPsolver.h
        include/Optimizer.h
        include/GeometricTools.h
        include/ORBextractor.h
        include/LoopClosing.h
        include/OptimizableTypes.h
        include/Sim3Solver.h
        include/ORBmatcher.h
        include/Tracking.h
        include/System.h
        include/CameraModels/Pinhole.h
        include/CameraModels/KannalaBrandt8.h
        include/CameraModels/GeometricCamera.h
        include/Atlas/Atlas.h
        include/Atlas/KeyFrame.h
        include/Atlas/KeyFrameDatabase.h
        include/Atlas/MapPoint.h
        include/Atlas/Map.h
        include/Visualise/FrameDrawer.h
        include/Visualise/MapDrawer.h
        include/Visualise/TwoViewReconstruction.h
        include/Visualise/Viewer.h
        include/Visualise/ORBVocabulary.h
        include/Visualise/SerializationUtils.h
        include/Visualise/ImuTypes.h
        include/Visualise/G2oTypes.h
        include/Visualise/Converter.h
        )

add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)

target_link_libraries(ORB_SLAM3
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        # whats up
        ${PROJECT_SOURCE_DIR}/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        -lboost_serialization
        -lcrypto
        )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc)

target_link_libraries(stereo_kitti ORB_SLAM3)
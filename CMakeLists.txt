cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 14)
project(ObjectDetectionAndTracking)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14 -pthread -g")

include_directories(include 
                    ${PROJECT_SOURCE_DIR}/include 
                    ${PROJECT_SOURCE_DIR}/include/Detector 
                    ${PROJECT_SOURCE_DIR}/include/Tracker
                    )

find_package(OpenCV 4.1 REQUIRED)
find_package (Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(detect_and_track src/main.cpp 
                                src/frame_grabber.cpp 
                                src/yolo_object_detector.cpp 
                                src/frame_writer.cpp
                                src/hungarian.cpp
                                src/kalman_filter.cpp
                                src/kalman_track.cpp
                                src/tracker.cpp
                                src/object_tracker.cpp
                                )

target_link_libraries(detect_and_track ${OpenCV_LIBRARIES} ${EIGEN3_DIR})

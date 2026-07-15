cmake_minimum_required(VERSION 3.10)
project(ArUcoDocking)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR "E:/Libraries/OpenCV_Windows/build")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ArUco_App src/single_marker_tracker.cpp)
target_link_libraries(ArUco_App ${OpenCV_LIBS})

add_executable(GridTracker src/grid_tracker.cpp)
target_link_libraries(GridTracker ${OpenCV_LIBS})

add_executable(Calibrator src/calibration.cpp)
target_link_libraries(Calibrator ${OpenCV_LIBS})


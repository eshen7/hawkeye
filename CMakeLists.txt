cmake_minimum_required(VERSION 3.22.1)

project(Hawkeye)

set(CMAKE_CXX_FLAGS "-std=c++17 -Ofast -Wall")

find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-audio-1.0 gstreamer-video-1.0  gstreamer-app-1.0)

link_directories(${GST_LIBRARY_DIRS})
include_directories(${GST_INCLUDE_DIRS})

include_directories(
    include
    ${GTSAM_INCLUDE_DIRS}
    )

find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)

add_executable(${PROJECT_NAME} src/Hawkeye.cpp src/RobotTracker.cpp src/TargetMap.cpp src/TrackedTarget.cpp)

target_link_libraries(${PROJECT_NAME} gtsam)
target_link_libraries(${PROJECT_NAME} ${GST_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION bin)
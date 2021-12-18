cmake_minimum_required(VERSION 3.21)
project(dh_camera_demo)

set(CMAKE_CXX_STANDARD 20)

set(DH_CAM_PATH /home/trantuan/Galaxy_camera)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${DH_CAM_PATH}/inc)
file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/x86_64/*.so)

include_directories(/usr/include/x86_64-linux-gnu)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(dh_camera_demo ${SRC})

target_link_libraries(dh_camera_demo
        ${DH_LIBS}
        ${OpenCV_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
        )

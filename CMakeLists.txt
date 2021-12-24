cmake_minimum_required(VERSION 3.21)
project(dh_camera_demo)

set(CMAKE_CXX_STANDARD 20)

set(DH_CAM_PATH /home/trantuan/Galaxy_camera)

add_compile_options(
        -std=c++2a
        -DELPP_THREAD_SAFE
        -DELPP_UNICODE
)

# Options for different build type.
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_compile_options(
            # Complete debugger support.
            -O0
            # Default log file path.
            -DELPP_DEFAULT_LOG_FILE="../log/debug.log"
            # Enable stack backtrace log when the program crashes.
            -DELPP_FEATURE_CRASH_LOG
            # Enable performance tracking macro: TIMED_SCOPE, TIMED_FUNC, etc.
            -DELPP_FEATURE_PERFORMANCE_TRACKING
    )
elseif (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(
            # Maximum speed.
            -O3
            -flto
            # Disable this flag when error occurs.
            -march=native
            -DELPP_DEFAULT_LOG_FILE="../log/release.log"
            # Disable LOG(DEBUG) to improve performance.
            -DELPP_DISABLE_DEBUG_LOGS
    )
endif ()

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${DH_CAM_PATH}/inc)
file(GLOB DH_LIBS ${DH_CAM_PATH}/lib/x86_64/*.so)

include_directories(/usr/include/x86_64-linux-gnu)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

file(GLOB LOGGING_SRC ${CMAKE_CURRENT_SOURCE_DIR}/easylogging++/*.c*)

add_executable(dh_camera_demo ${SRC} ${LOGGING_SRC})

target_link_libraries(dh_camera_demo
        ${DH_LIBS}
        ${OpenCV_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
        )

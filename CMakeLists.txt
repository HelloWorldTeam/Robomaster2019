cmake_minimum_required(VERSION 3.5)
project(RoboMasterShaoBing_new)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")
find_package(OpenCV 3 REQUIRED)
message(STATUS "opencv version " ${OpenCV_VERSION})
message(STATUS "opencv directories " ${OpenCV_DIR})

add_definitions(-D__SHOW_DEBUGINFO__)
#add_definitions(-D__SHOWING_SRC_IMG__)
#add_definitions(-D__SHOWING_ALL_IMG__)
#add_definitions(-D__SAVE_VIDEO__)
#add_definitions(-D__NO_SOFT_TRIG__)
#add_definitions(-D__READ_VIDEO__)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/MVCamera/include
        ${PROJECT_SOURCE_DIR}/TargetCarDetector/include
)

set(SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/RemoteControl.cpp
        ${PROJECT_SOURCE_DIR}/src/uart.cpp
        ${PROJECT_SOURCE_DIR}/src/RoboMasterProcess.cpp
        ${PROJECT_SOURCE_DIR}/src/can.cpp
        ${PROJECT_SOURCE_DIR}/MVCamera/src/MVCamera.cpp
        ${PROJECT_SOURCE_DIR}/TargetCarDetector/src/Params.cpp
        ${PROJECT_SOURCE_DIR}/TargetCarDetector/src/TargetCarDetector.cpp
        )

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

add_executable(main main.cpp ${SOURCE_FILES})
if( ${ARCHITECTURE} STREQUAL "x86_64" )
    target_link_libraries(main ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/MVCamera/lib/x64/libMVSDK.so)
else()
    target_link_libraries(main ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/MVCamera/lib/arm64/libMVSDK.so)
endif()

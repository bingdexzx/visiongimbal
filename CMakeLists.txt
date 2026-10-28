cmake_minimum_required(VERSION 3.10)

project(VisionGimbal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)
find_package(Threads REQUIRED)

add_subdirectory(Serial)

add_executable(vision_gimbal vision_gimbal.cpp)
target_include_directories(vision_gimbal PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_gimbal PRIVATE
    ${OpenCV_LIBS} TRoMaC::serial Threads::Threads)

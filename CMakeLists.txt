cmake_minimum_required(VERSION 3.12)
project(EricCV)
set(CMAKE_CXX_STANDARD 11)

set(target_list)

set(calib_src calib.cpp)
add_executable(calibration ${calib_src})
list(APPEND target_list calibration)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message("opencv found")
    include_directories(OpenCV_INCLUDE_DIRS)
    foreach(target ${target_list})
        target_link_libraries(${target} ${OpenCV_LIBS})
    endforeach()
else(OpenCV_FOUND)
    message("cannot found opencv")
endif(OpenCV_FOUND)

add_executable(EricCV main.cpp)
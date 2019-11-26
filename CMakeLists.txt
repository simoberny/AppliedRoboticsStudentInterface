## Minimum version of cmake
cmake_minimum_required(VERSION 2.8.3)

## Name of the project
project(student_project)

## Compile as C++11
add_compile_options(-std=c++11)

## Required packages
find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)

## Add executables
add_executable(calibrate
  src/camera_calibration.cpp
  )
add_executable(test
  src/test_envelop.cpp
  )


## Add executables
add_executable(test1
  src/test_dubins.cpp
  )

  ## Add shared library
add_library(dubins SHARED
   src/dubins.cpp
)


## Add shared library
add_library(student SHARED
   src/student_interface.cpp
   src/find_robot.cpp
   src/process_arena.cpp
   src/find_victim.cpp
   src/dubins.cpp
   src/loadImage.cpp
        src/find_collision.cpp src/include/find_collision.hpp
        src/Voronoi.cpp src/include/Voronoi.hpp)

## Link with libraries 
target_link_libraries(student
  ${OpenCV_LIBRARIES}
)

target_link_libraries(calibrate
  ${OpenCV_LIBRARIES}  
)

target_link_libraries(test1
  dubins
)


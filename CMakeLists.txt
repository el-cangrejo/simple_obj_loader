cmake_minimum_required(VERSION 2.8)
project(Objloader)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(include)
include_directories(${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

file(GLOB SOURCES "src/*.cpp")

add_executable(Objloader.out ${SOURCES})

target_link_libraries(Objloader.out ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

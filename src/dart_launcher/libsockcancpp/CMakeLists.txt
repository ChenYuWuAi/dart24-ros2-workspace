CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
PROJECT(socketcancpp)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/exceptions)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

if(BUILD_SHARED_LIBS)
    add_library(socketcancpp SHARED src/CanDriver.cpp)
else()
    add_library(socketcancpp STATIC src/CanDriver.cpp)
endif()

# make install 安装在本目录下，不会安装到系统目录
install(TARGETS socketcancpp
    LIBRARY DESTINATION ${CMAKE_CURRENT_LIST_DIR}/lib
    PUBLIC_HEADER DESTINATION ${CMAKE_CURRENT_LIST_DIR}/include)
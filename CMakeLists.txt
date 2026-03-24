cmake_minimum_required(VERSION 3.10)
project(embedded_abs_ecu VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include directory for headers
include_directories(include)

# add executable with all source files
file(GLOB SOURCES "src/*.cpp")   # src altındakı bütün cpp faylları avtomatik tapır
add_executable(abs_ecu_sim main.cpp ${SOURCES})
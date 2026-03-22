cmake_minimum_required(VERSION 3.10)
project(embedded_abs_ecu VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(abs_ecu_sim abs_ecu_sim.cpp)

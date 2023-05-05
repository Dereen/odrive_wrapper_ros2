# Copyright (c) JettyVision s.r.o in Prague  - All Rights Reserved
# Created on: 21.5.2021
#     Author: Valentyn Cihala <cihala@jettyvision.cz>

cmake_minimum_required(VERSION 2.8.12)
project(odrivecan)

add_compile_options(-std=c++11 -Wall -pthread)

include_directories(include)

add_library(${PROJECT_NAME} src/candevice.cpp src/odrivecan.cpp src/odriveaxis.cpp)
target_link_libraries(${PROJECT_NAME} -pthread)

add_executable(odrivecan_test src/main.cpp)
target_link_libraries(odrivecan_test ${PROJECT_NAME})

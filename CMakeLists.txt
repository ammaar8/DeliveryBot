cmake_minimum_required(VERSION 3.0.0)
project(DeliveryBot VERSION 0.1.0)

include(CTest)
enable_testing()

add_library(DeliveryBot DeliveryBot.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

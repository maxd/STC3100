cmake_minimum_required(VERSION 3.17)

set(EXTRA_COMPONENT_DIRS "../components")
set(TEST_COMPONENTS "STC3100" CACHE STRING "List of components to test")

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(EspIdf)
find_package(Espressif)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(STC3100_Test)
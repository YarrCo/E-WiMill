# Top-level CMake file for the E-WiMill project
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(EWiMill)

# Make TinyUSB pick up project-level tusb_config.h
idf_build_set_property(INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/main" APPEND)

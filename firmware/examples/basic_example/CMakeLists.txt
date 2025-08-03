# Basic Example for ESP32 MCP Bridge

cmake_minimum_required(VERSION 3.16)

# Set the name of the project
set(PROJECT_NAME "basic_example")

# Include the ESP-IDF CMake project
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(${PROJECT_NAME})

# Optional: Set component directories if needed
# set(EXTRA_COMPONENT_DIRS "../../../components") 
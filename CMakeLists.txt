# mbed-cmake requires at least CMake 3.12
cmake_minimum_required(VERSION 3.12)

# load project upload methods (must be before mbed-cmake is loaded)
# -------------------------------------------------------------
include(cmake/UploadMethodConfig.cmake)

# load compilers and flags
# -------------------------------------------------------------

include(mbed-cmake/mbed-cmake.cmake)

project(MBED-TEST LANGUAGES NONE)

include_directories(include)

add_subdirectory(drivers)

add_mbed_executable(mbed-test main.cpp)

target_link_libraries(mbed-test ADS124S0x)

# build report
# -------------------------------------------------------------

mbed_cmake_print_build_report()

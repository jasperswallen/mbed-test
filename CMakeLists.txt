# mbed-cmake requires at least CMake 3.12
cmake_minimum_required(VERSION 3.12)

# load project upload methods (must be before mbed-cmake is loaded)
# -------------------------------------------------------------
include(cmake/UploadMethodConfig.cmake)

# load compilers and flags
# -------------------------------------------------------------

include(mbed-cmake/mbed-cmake.cmake)

project(MBED-TEST LANGUAGES NONE)

add_subdirectory(ScopeGuard)
add_subdirectory(drivers)

include_directories(include)

add_library(libmbed-test INTERFACE)
target_link_libraries(libmbed-test INTERFACE kx134 ubloxgps lm75b ms5607 adis16467 bno080)

add_mbed_executable(mbed-test main.cpp)

target_link_libraries(mbed-test libmbed-test)

# build report
# -------------------------------------------------------------

mbed_cmake_print_build_report()

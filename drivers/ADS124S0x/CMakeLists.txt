cmake_minimum_required(VERSION 3.14)

set(ADS124S0x_SOURCES
        ADS124S0x.h
        ADS124S0x.cpp)

set(ADS124S0x_TEST_SOURCES
        TestADS124S0x.cpp)

# ADS124S0x ADC Library
add_library(ADS124S0x STATIC ${ADS124S0x_SOURCES})
target_include_directories(ADS124S0x PUBLIC .)
target_link_libraries(ADS124S0x mbed-os)

# test program
add_mbed_executable(test_ADS124S0x ${ADS124S0x_TEST_SOURCES})
target_link_libraries(test_ADS124S0x ADS124S0x)

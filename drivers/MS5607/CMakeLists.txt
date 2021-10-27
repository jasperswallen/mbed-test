add_library(ms5607 INTERFACE) # header only library, so just make interface target
target_include_directories(ms5607 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ms5607 INTERFACE mbed-os)

add_library(lm75b LM75B.cpp)
target_include_directories(lm75b PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lm75b mbed-os)
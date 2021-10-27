add_library(adis16467 ADIS16467.cpp)
target_include_directories(adis16467 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(adis16467 mbed-os)
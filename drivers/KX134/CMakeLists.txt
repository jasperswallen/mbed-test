add_library(kx134 KX134Base.cpp KX134I2C.cpp KX134SPI.cpp)
target_include_directories(kx134 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kx134 mbed-os)

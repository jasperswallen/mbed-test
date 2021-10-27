add_library(bno080 BNO080.cpp)

target_include_directories(bno080 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(bno080 PUBLIC cxx_std_14)
target_link_libraries(bno080 mbed-os)

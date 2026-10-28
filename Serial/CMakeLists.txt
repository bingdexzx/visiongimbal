# 串口库：Linux termios 实现，不依赖第三方串口库。
add_library(tromac_serial STATIC Serial.cpp)
add_library(TRoMaC::serial ALIAS tromac_serial)

target_compile_features(tromac_serial PUBLIC cxx_std_11)
target_compile_options(tromac_serial PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

target_include_directories(tromac_serial PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

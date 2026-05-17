cmake_minimum_required(VERSION 3.16)

# C++용 표준
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=gnu++26>)

# C용 표준 (보통 gnu11 또는 gnu17 사용)
add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=gnu17>)

add_compile_options(
    -Wno-deprecated-declarations
    -Wno-address-of-packed-member
    #-O3 #성능 극대화 (실제 사용할때...)
    #-Os  #Optimize for Size
    -Og  #Optimize for Debugging (개발시)
    -ffast-math
    -fno-finite-math-only
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(ESP32S3_DRONE)


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
set(IDF_COMPONENT_VALIDATION_LEVEL "WARN_NOT_SET")
# 💡 [핵심 추가] 내 컴포넌트들이 모여있는 폴더 경로를 빌드 시스템에 강제로 주입합니다.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(ESP32S3_DRONE)

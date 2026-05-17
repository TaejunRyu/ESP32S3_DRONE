#file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS #${SOURCES}
        "ryu_spi.cpp"
    INCLUDE_DIRS 
            "."  
    REQUIRES
        esp_driver_gpio
        esp_driver_spi    
)
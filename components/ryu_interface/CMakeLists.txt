file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS 
        "."  

    REQUIRES
        esp_timer 
        esp_driver_gpio
        esp_driver_spi
        esp_driver_i2c
        ryu_control
    
    
)
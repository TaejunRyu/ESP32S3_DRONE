file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS ${SOURCES}

    INCLUDE_DIRS 
        "."  
    REQUIRES
        esp_timer
        esp_event
        esp_driver_gpio
        esp_driver_mcpwm
        esp_pm
        esp_wifi
        nvs_flash
        driver
        freertos
        mavlink
        ryu_service
        ryu_system
        ryu_sensor
    
)
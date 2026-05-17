# ryu_service component
#file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS #${SOURCES}
        "ryu_BaroSensorTask.cpp"
        "ryu_espnow.cpp"
        "ryu_SensorTask.cpp"
        
    INCLUDE_DIRS 
        "."  

    REQUIRES
        esp_timer
        esp_wifi
        freertos
        nvs_flash
        mavlink
        ryu_interface
        ryu_sensor
        ryu_util    
)
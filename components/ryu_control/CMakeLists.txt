#file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS #${SOURCES}
        "ryu_FlightTask.cpp"
        
    INCLUDE_DIRS 
        "."
    REQUIRES 
        esp_timer
        freertos
        ryu_system
        ryu_sensor
        ryu_driver
        ryu_util
        ryu_filter
        ryu_service
    
)
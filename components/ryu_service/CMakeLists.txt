# ryu_service component
file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS ${SOURCES}       
        
    INCLUDE_DIRS 
        "."  

    PRIV_REQUIRES
        esp_timer
        esp_wifi
        freertos
        nvs_flash
        ryu_interface
        ryu_sensor
        ryu_util    
        mavlink
)


file(GLOB_RECURSE SOURCES "*.cpp") 

idf_component_register(
    SRCS ${SOURCES}

    INCLUDE_DIRS 
        "."  

    REQUIRES
        esp_timer 
        esp_driver_gpio
        esp_driver_spi
        esp_driver_i2c

)
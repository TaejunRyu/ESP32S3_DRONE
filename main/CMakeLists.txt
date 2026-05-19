idf_component_register(
    SRCS 
        "main.cpp"
                    
    INCLUDE_DIRS "."

    REQUIRES 
        esp_event
        nvs_flash
        ryu_controller

)

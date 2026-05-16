idf_component_register(
    SRCS 
        "main.cpp"
                    
    INCLUDE_DIRS "."

    REQUIRES 
        ryu_control
        ryu_sensor
        ryu_driver
        ryu_service
)

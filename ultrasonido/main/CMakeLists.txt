idf_component_register(
    SRCS "hello_world_main.c" "ultrasonido.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component
             esp_driver_gpio
             esp_driver_rmt
)

idf_component_register(
    SRCS "hello_world_main.c" "laser.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component vl53l0x esp_driver_i2c
)

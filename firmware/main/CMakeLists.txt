idf_component_register(SRCS "main.cpp" "peripherals.cpp" "imu.cpp" "world.cpp" "vision.cpp" "distance.cpp"
                       PRIV_REQUIRES esp_driver_i2c esp_driver_uart esp_driver_mcpwm esp_driver_ledc esp_driver_gpio esp_timer                       INCLUDE_DIRS ".")

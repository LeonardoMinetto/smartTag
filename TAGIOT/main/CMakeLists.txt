idf_component_register(
    SRCS "main.c" "mpu6050_impact.c" "gps_neo_m8n.c" "mqtt_telemetry.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_driver_uart esp_driver_i2c esp_driver_gpio nvs_flash esp_wifi esp_netif esp_event
)
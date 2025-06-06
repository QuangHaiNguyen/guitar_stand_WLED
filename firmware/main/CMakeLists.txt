idf_component_register(
    SRCS "main.c"
    REQUIRES app_wifi_manager
    PRIV_REQUIRES esp_event nvs_flash esp_wifi esp_http_server
    INCLUDE_DIRS ".")

target_link_libraries(${COMPONENT_LIB} PRIVATE
    easy_embedded_lib
    app_gpio_lib
    app_tof_lib
    app_event_bus_lib
)

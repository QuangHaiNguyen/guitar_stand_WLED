idf_component_register(
    SRCS "app_color_picker.c"
    PRIV_REQUIRES esp_http_server esp_wifi
    EMBED_FILES color_picker.html
    INCLUDE_DIRS "." "../../")

target_link_libraries(${COMPONENT_LIB} PRIVATE
    easy_embedded_lib
    app_event_bus_lib
)
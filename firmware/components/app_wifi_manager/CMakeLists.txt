idf_component_register(
    SRCS "app_wifi_manager.c"
    PRIV_REQUIRES esp_event nvs_flash esp_wifi esp_http_server
    EMBED_FILES captive_portal.html
    INCLUDE_DIRS "." "../../")

target_link_libraries(${COMPONENT_LIB} PRIVATE
    easy_embedded_lib
)

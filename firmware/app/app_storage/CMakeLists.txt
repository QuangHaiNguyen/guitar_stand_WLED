

add_library(app_storage_lib STATIC app_storage.c)

target_include_directories(app_storage_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/main
)

target_link_libraries(app_storage_lib PRIVATE
    easy_embedded_lib
    app_event_bus_lib
    idf::nvs_flash
)


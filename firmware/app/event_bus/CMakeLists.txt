add_library(app_event_bus_lib STATIC "app_event_bus.c")

target_include_directories(app_event_bus_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/main
)

target_link_libraries(app_event_bus_lib PRIVATE
    easy_embedded_lib
    idf::freertos
)
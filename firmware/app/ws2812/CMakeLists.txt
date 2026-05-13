

add_library(ws2812_lib STATIC ws2812.c)

target_include_directories(ws2812_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/main
)

target_link_libraries(ws2812_lib PRIVATE
    easy_embedded_lib
    app_event_bus_lib
    idf::espressif__led_strip
)


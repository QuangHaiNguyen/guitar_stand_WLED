add_library(app_gpio_lib STATIC "app_gpio.c")

target_include_directories(app_gpio_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/main
)

target_link_libraries(app_gpio_lib PRIVATE
    easy_embedded_lib
    gpio_port_lib
    idf::freertos
)
add_library(gpio_port_lib STATIC "gpio_port.c")

target_include_directories(gpio_port_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/main
)

target_link_libraries(gpio_port_lib PRIVATE
    easy_embedded_lib
    idf::esp_driver_gpio
)
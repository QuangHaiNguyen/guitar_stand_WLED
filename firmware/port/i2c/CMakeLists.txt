add_library(i2c_port_lib STATIC "port_i2c.c")

target_include_directories(i2c_port_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/main
)

target_link_libraries(i2c_port_lib PRIVATE
    easy_embedded_lib
    idf::driver
)
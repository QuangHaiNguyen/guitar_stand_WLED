add_library(app_tof_lib STATIC "app_tof.c")

target_include_directories(app_tof_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/main
)

file(GLOB_RECURSE API_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Api/**/*.c")
file(GLOB_RECURSE API_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/Api/**/*.h")

message(STATUS "API_SOURCES: ${API_SOURCES}")
message(STATUS "API_HEADERS: ${API_HEADERS}")

add_library(vl53l0x_lib STATIC ${API_SOURCES})

target_include_directories(vl53l0x_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Api/core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Api/platform/inc
)

target_link_libraries(vl53l0x_lib PRIVATE
    easy_embedded_lib
    i2c_port_lib
    idf::freertos
)

#target_sources(vl53l0x_lib PRIVATE ${API_SOURCES} ${API_HEADERS})

target_link_libraries(app_tof_lib PRIVATE
    easy_embedded_lib
    i2c_port_lib
    vl53l0x_lib
    app_event_bus_lib
    idf::freertos
)


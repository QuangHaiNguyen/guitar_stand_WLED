idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

target_link_libraries(${COMPONENT_LIB}
    easy_embedded_lib
)

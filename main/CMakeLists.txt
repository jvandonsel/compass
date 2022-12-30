idf_component_register(SRCS 
                    "compass_main.cpp"
                    INCLUDE_DIRS "include"
                    EMBED_FILES ../static/index.html)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
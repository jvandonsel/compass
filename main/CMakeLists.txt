idf_component_register(SRCS 
                    "compass_main.cpp"
                    "magneto.cpp"
                    "gps.cpp"
                    "servo.cpp"
                    "util.cpp"
                    INCLUDE_DIRS "include")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

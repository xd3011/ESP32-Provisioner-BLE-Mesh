set(srcs "main.c"
        "app_mqtt.c")

idf_component_register(SRCS "app_mqtt.c" "${srcs}"
                    INCLUDE_DIRS  ".")

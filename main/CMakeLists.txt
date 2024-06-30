set(srcs "board.c" "main.c"
        "app_mqtt.c")

idf_component_register(SRCS "ap_mode.c" "board.c" "app_mqtt.c" "${srcs}"
                    INCLUDE_DIRS  ".")

idf_component_register(SRCS "mqtt_publisher.c"
                    SRCS "potentiometre.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES protocol_examples_common
                    REQUIRES mqtt
                    REQUIRES "driver"
                    )
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

idf_component_register(SRCS "mqtt_subscriber.c"
                    INCLUDE_DIRS "include"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES protocol_examples_common
                    REQUIRES mqtt
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")


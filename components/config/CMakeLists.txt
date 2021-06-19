idf_component_register(SRCS "config.c"
                       INCLUDE_DIRS "."
                       REQUIRES ""
                       PRIV_REQUIRES "freertos" "nvs_flash" "mpack")
idf_component_register(SRCS "flow.c"
                       INCLUDE_DIRS "."
                       REQUIRES "driver"
                       PRIV_REQUIRES "sensors" "freertos")
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#example-component-requirements
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#cmake-component-register
idf_component_register(SRCS "adder.c"
                       INCLUDE_DIRS "."
                       REQUIRES ""
                       PRIV_REQUIRES "")
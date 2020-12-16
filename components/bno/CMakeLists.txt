idf_component_register(SRCS "bno.c" "./sh2/sh2.c" "./sh2/sh2_util.c" "./sh2/shtp.c" "./sh2/sh2_SensorValue.c"
                       INCLUDE_DIRS "./sh2" "."
                       REQUIRES "driver"
                       PRIV_REQUIRES "sensors" "freertos" "i2c")
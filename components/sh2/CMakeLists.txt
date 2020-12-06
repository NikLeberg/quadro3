idf_component_register(SRCS "./bno080-driver/sh2.c" "./bno080-driver/sh2_util.c" "./bno080-driver/shtp.c" "./bno080-driver/sh2_SensorValue.c"
                       INCLUDE_DIRS "./bno080-driver" "."
                       REQUIRES "sensors"
                       PRIV_REQUIRES "")
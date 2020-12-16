/**
 * @file main.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief quadro3
 * @version 0.1
 * @date 2020-12-07
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * @note gdb variablen sehen: p 'datei.c'::variable
 * @note gdb task-backtrace sehen: thread apply all where
 * @note gdb buffer ansehen: x /länge pointer[hex]
 * @note Status: info program
 * @note Threads: info threads
 * @note zu Thread wechseln: thread X [Id gemäss vorheriger Liste]
 * 
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "i2c.h"
#include "sensors.h"
#include "bno.h"
#include "flow.h"



void app_main(void) {
    printf("Hallo quadro3!\n");

    printf("starte i2c: %u\n", i2cStart());

    printf("starte sensors: %u\n", sensorsStart());

    printf("starte bno: %u\n", bnoStart());

    printf("starte flow: %u\n", flowStart());
    
    int64_t lastTimestamp = 0;
    sensorsData_t state = {0};
    while (true) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        lastTimestamp = state.timestamp;
        sensorsGet(SENSORS_STATE_ORIENTATION, SENSORS_ENU_LOCAL, &state);
        printf("time:\t%llu\t%llu\n", state.timestamp, state.timestamp - lastTimestamp);
        printf("orient:\t%f\t%f\t%f\t%f\n", state.quaternion.real, state.quaternion.i, state.quaternion.j, state.quaternion.k);
        sensorsGet(SENSORS_STATE_EULER, SENSORS_ENU_WORLD, &state);
        printf("euler:\t%f\t%f\t%f\n", state.vector.x, state.vector.y, state.vector.z);
        sensorsGet(SENSORS_STATE_ACCELERATION, SENSORS_ENU_WORLD, &state);
        printf("accel:\t%f\t%f\t%f\n", state.vector.x, state.vector.y, state.vector.z);
        sensorsGet(SENSORS_STATE_ROTATION, SENSORS_ENU_WORLD, &state);
        printf("rotate:\t%f\t%f\t%f\n", state.vector.x, state.vector.y, state.vector.z);
    }
}

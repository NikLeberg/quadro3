/**
 * @file sensors.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Fusioniert alle Sensordaten und publiziert den Zustand des Quadros für andere Module.
 * @version 0.1
 * @date 2020-12-05
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * @note Daten werden im ENU Koordinatensystem erwartet und publiziert.
 * @note Jeder Sensor des Quadro registriert ein ProcessCallback. Dieser Callback wird entweder zyklisch oder
 * eventbasiert aufgerufen. Events lassen sich mit sensorsNotify(type) melden. Dem Callback wird ein leeres
 * sensorsData_t übergeben, welches die Callbackfunktion zu füllen hat.
 * 
 */

/**
 * @brief Includes
 * 
 */

#include "sensors.h"
#include "freertos/task.h"
#include "freertos/timers.h"


/**
 * @brief Typdeklarationen
 * 
 */

/* ... */


/**
 * @brief Variablendeklarationen
 * 
 */

static struct {
    TaskHandle_t taskHandle;

    struct {
        sensorsProcessCallback_t callback; // registrierter Callback
        TimerHandle_t timer; // falls nicht eventbasiert: Softwaretimer der manuell sensorsNotify() aufruft
        sensorsData_t voidData; // leere Datenstruktur die dem Callback übergeben wird
        sensorsData_t rawData; // rohe Sensordaten, Kopie von voidData nachdem der Callback fertig ist
    } rawSensors[SENSORS_MAX];
} sensors;


/**
 * @brief Private Funktionsprototypen
 * 
 */

/**
 * @brief Callback für Software-Timer.
 * Für Sensoren die nicht eventbasiert sondern per intervall arbeiten, wurde in sensorsRegister() ein Timer erstellt.
 * Läuft dieser Timer ab wird in diesem Callback manuell ein sensorsNotify() für den entsprechenden Sensor ausgelöst.
 * 
 * @param expiredTimer Handle des abgelaufenen Timers.
 */
static void sensorsTimerCallback(TimerHandle_t expiredTimer);


/**
 * @brief Implementation Öffentlicher Funktionen
 * 
 */

bool sensorsRegister(sensorsType_t type, sensorsProcessCallback_t callback, TickType_t intervall) {
    if (type >= SENSORS_MAX || !callback) return true;
    sensors.rawSensors[type].callback = callback;
    if (intervall) {
        sensors.rawSensors[type].timer = xTimerCreate("", intervall, pdTRUE, &sensors.rawSensors[type].timer, &sensorsTimerCallback);
        if (!sensors.rawSensors[type].timer) {
            return true;
        }
    }
    return false;
}

void sensorsNotify(sensorsType_t type) {
    assert(type < SENSORS_MAX);
    sensors.rawSensors[type].voidData.timestamp = esp_timer_get_time();
    xTaskNotify(sensors.taskHandle, 0x1 << type, eSetBits);
}
void sensorsNotifyFromISR(sensorsType_t type) {
    assert(type < SENSORS_MAX);
    sensors.rawSensors[type].voidData.timestamp = esp_timer_get_time();
    BaseType_t woken;
    xTaskNotifyFromISR(sensors.taskHandle, 0x1 << type, eSetBits, &woken);
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}


/**
 * @brief Implementation Privater Funktionen
 * 
 */

static void sensorsTimerCallback(TimerHandle_t expiredTimer) {
    for (sensorsType_t type = 0; type < SENSORS_MAX; ++type) {
        if (sensors.rawSensors[type].timer == expiredTimer) {
            sensorsNotify(type);
            break;
        }
    }
    assert(false);
}
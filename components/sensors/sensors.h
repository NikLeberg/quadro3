/**
 * @file sensors.h
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

#pragma once


/**
 * @brief Includes
 * 
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"


/**
 * @brief Typdeklarationen
 * 
 */

/**
 * @brief Unterstützte Sensortypen
 * 
 */
typedef enum {
    SENSORS_ACCELERATION, // m/s^2 [vector]
    SENSORS_ORIENTATION, // Ausrichtung [quaternion]
    SENSORS_ROTATION, // rad/s [vector]
    SENSORS_ALTIMETER, // umgerechneter Druck Pa -> m [vector->z]
    SENSORS_POSITION, // GPS Fix lat/lon/alt [vector]
    SENSORS_GROUNDSPEED, // m/s [vector]
    SENSORS_VOLTAGE, // V [value]
    SENSORS_OPTICAL_FLOW, // rad/s [vector]
    SENSORS_LIDAR, // m [-vector->z]
    SENSORS_MAX
} sensorsType_t;

/**
 * @brief Generischer Vektor
 * 
 */
typedef struct {
    union {
        struct {
            float x, y, z;
        };
        float v[3];
    };
} sensorsVector_t;

/**
 * @brief Quaternion
 * 
 */
typedef struct {
    float i, j, k;
    float real;
} sensorsQuaternion_t;

/**
 * @brief Sensordatenpunkt
 * 
 */
typedef struct {
    int64_t timestamp;
    union {
        float value;
        sensorsVector_t vector;
        sensorsQuaternion_t quaternion;
    };
} sensorsData_t;

/**
 * @brief Pointer zu einem Verarbeitungscallback
 * 
 * @param sensorsData_t leerer Sensordatenpunkt welcher der Callback auszufüllen hat.
 * @return true - Fehlgeschlagen und Datenpunkt nicht verwenden, false - Erfolgreich
 */
typedef bool (*sensorsProcessCallback_t)(*sensorsData_t);


/**
 * @brief Variablendeklarationen
 * 
 */

/* ... */


/**
 * @brief Öffentliche Funktionen
 * 
 */

/**
 * @brief Initialisiere Sensors-Task und Fusionsalgorithmus.
 * 
 * @return true - Initialisierung fehlgeschlagen, false - Initialisierung erfolgreich
 */
bool sensorsInit();

/**
 * @brief Registriere ein Sensor.
 * 
 * @param type Sensortyp. Jeder Typ kann nur einmal gleichzeitig existieren.
 * @param callback Callback welcher ausgeführt wird wenn ein Notify erhalten wird, oder die Intervallzeil abgelaufen ist.
 * @param intervall Intervall resp. Frequenz mit der der Callback aufgerufen werden soll. 0 wenn der Sensor eventbasiert ist.
 * @return true - Registrierung nicht erfolgreich, false - Registrierung erfolgreich
 */
bool sensorsRegister(sensorsType_t type, sensorsProcessCallback_t callback, TickType_t intervall);

/**
 * @brief Sende ein Sensorevent.
 * 
 * @param type Sensortyp der die Benachrichtigung auslöst.
 */
void sensorsNotify(sensorsType_t type);
void sensorsNotifyFromISR(sensorsType_t type);

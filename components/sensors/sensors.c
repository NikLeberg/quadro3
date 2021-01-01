/**
 * @file sensors.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Fusioniert alle Sensordaten und publiziert den Zustand des Quadros für andere Module.
 * @version 0.1
 * @date 2020-12-05
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * - Daten werden im ENU Koordinatensystem erwartet und publiziert.
 * - Jeder Sensor des Quadro registriert ein ProcessCallback. Dieser Callback wird entweder zyklisch oder
 * eventbasiert aufgerufen. Events lassen sich mit sensorsNotify() melden.
 * - Hat ein Sensor neue Messwerte übergibt er diese per sensorsSetRaw().
 * - Messwerte werden verarbeitet, ggf. von ENU-lokal in world oder umgekehrt umgerechnet, ggf. fusioniert und dann als Stats abgespeichert.
 * - Components welche Daten benötigen können diese per sensorsGetState() erhalten.
 * 
 */

/**
 * @brief Includes
 * 
 */

#include "sensors.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>


/**
 * @brief Typdeklarationen
 * 
 */

#define FAIL_DELAY  (10 / portTICK_PERIOD_MS)

/**
 * @brief Speicher für Sensorregistrierungen
 * 
 */
typedef struct {
    sensorsProcessCallback_t callback; // registrierter Callback
    void *cookie; // optionaler Cookie für Callback
    TimerHandle_t timer; // falls nicht eventbasiert: Softwaretimer der manuell sensorsNotify() aufruft
    sensorsData_t data; // rohe Sensordaten
    bool needsProcessing; // markiere für Nacharbeit, denn Daten wurden empfangen müssen aber noch fusioniert usw. werden
} rawSensor_t;


/**
 * @brief Variablendeklarationen
 * 
 */

static struct {
    TaskHandle_t taskHandle; // wenn Handle != NULL ist System aktiv

    struct {
        SemaphoreHandle_t lock; // Zugriffsschutz für Rohdaten
        rawSensor_t sensors[SENSORS_MAX];
    } raw;

    struct {
        SemaphoreHandle_t lock; // Zugriffsschutz für Statusdaten
        sensorsData_t data[SENSORS_STATE_MAX][SENSORS_ENU_MAX]; // Speicher für alle möglichen Stati
    } state;

} sensors;


/**
 * @brief Private Funktionsprototypen
 * 
 */

/**
 * @brief Sensor-Task
 * Wartet auf Notifys ausgelöst durch Sensoren oder über Softwaretimer und ruft die entsprechenden Callbacks auf.
 * Je nach Sensortyp wird danach der Fusionsalgorithmus gerechnet und der Systemstatus aktualisiert.
 * 
 * @param args per xTaskCreate() übergebene Argumente.
 */
static void sensorsTask(void *args);

/**
 * @brief Callback für Software-Timer.
 * Für Sensoren die nicht eventbasiert sondern per intervall arbeiten, wurde in sensorsRegister() ein Timer erstellt.
 * Läuft dieser Timer ab wird in diesem Callback manuell ein sensorsNotify() für den entsprechenden Sensor ausgelöst.
 * 
 * @param expiredTimer Handle des abgelaufenen Timers.
 */
static void timerCallback(TimerHandle_t expiredTimer);

/**
 * @brief Sperre Rohdaten der Sensoren für Änderungen.
 */
static void lockRaw();

/**
 * @brief Entsperre die Rohdaten.
 * 
 */
static void unlockRaw();

/**
 * @brief Sperre Status für Änderungen.
 */
static void lockState();

/**
 * @brief Entsperre den Status.
 * 
 */
static void unlockState();

/**
 * @brief Verarbeitung von Rohdaten.
 * Orientierung, Beschleunigung und Rotation werden in beide Referenzpunkte umgerechnet und in den Status kopiert.
 * Für Höhe über Meer, Optischer Fluss und Distanzmessung wird ENU-local garantiert. Sonst wird umgerechnet.
 * GPS Position und Groundspeed werden per assert auf ENU-world geprüft.
 * Batteriespannung wird nur kopiert.
 * @note state muss gesperrt sein
 * 
 * @param type welche Rohdaten zu verarbeiten sind
 */
static void processRaw(sensorsType_t type);

/**
 * @brief Verarbeite die einzelne Orientierung in Quaternionen mit denen ein Vektor von ENU lokal zu world und zurück
 * rotiert werden kann.
 * 
 * @param raw Rohdatenpunkt des Sensors
 * @param local Orientierungs-Quaternion von Local zu World
 * @param world Orientierungs-Quaternion von World zu Local
 */
static void processOrientation(sensorsData_t *raw, sensorsData_t *local, sensorsData_t *world);

/**
 * @brief Rechne / Rotiere ein Datenpunkt in beide Referenzsysteme.
 * 
 * @param raw Rohdatenpunkt des Sensors
 * @param local Datenpunkt im lokalen System
 * @param world Datenpunkt im welt System
 */
static void processVector(sensorsData_t *raw, sensorsData_t *local, sensorsData_t *world);

/**
 * @brief Rechne die Orientierung von einem Quaternion in Eulerwinkel um.
 * 
 * @note state muss gesperrt sein.
 */
static void quaternionToEuler();

/**
 * @brief Rotiere Vektor mittels gegebenem Quaternion.
 * 
 * @param vector zu rotierender Vektor.
 * @param q Ausrichtung als Quaternion.
 */
static void rotateVector(sensorsVector_t *vector, sensorsQuaternion_t *q);

/**
 * @brief Garantiere dass der Datenpunkt in der angegebenen Referenz liegt.
 * 
 * @param data Datenpunkt
 * @param reference zu garantierende Referenz
 */
static void ensureReference(sensorsData_t *data, sensorsENU_t reference);

/**
 * @brief Wähle aus wie der jeweilige Typ Fusioniert wird.
 * Für Z wird Beschleunigung, Höhe über Meer und Distanz zum Boden fusioniert.
 * Für X und Y wird jeweils GPS-Position, Beschleunigung und Optischer Fluss fusioniert.
 * 
 * @note state muss gesperrt sein.
 * @param type welche Daten empfangen wurden
 */
static void fuse(sensorsType_t type);


/**
 * @brief Implementation Öffentlicher Funktionen
 * 
 */

bool sensorsStart() {
    if (sensors.taskHandle) return true;
    // Locks erstellen
    sensors.raw.lock = xSemaphoreCreateMutex();
    sensors.state.lock = xSemaphoreCreateMutex();
    if (!sensors.raw.lock || !sensors.state.lock) return true;
    // Task starten
    if (!sensors.taskHandle) {
        if (xTaskCreate(sensorsTask, "sensors", 8 * 1024, NULL, 1, &sensors.taskHandle) != pdPASS) return true;
    }
    // konfigurierte Timer starten
    for (sensorsType_t type = 0; type < SENSORS_MAX; ++type) {
        if (sensors.raw.sensors[type].timer) {
            BaseType_t success;
            success = xTimerStart(sensors.raw.sensors[type].timer, FAIL_DELAY);
            assert(success == pdTRUE);
        }
    }
    return false;
}

void sensorsStop() {
    if (!sensors.taskHandle) return;
    lockRaw();
    lockState();
    // Timer stoppen
    for (sensorsType_t type = 0; type < SENSORS_MAX; ++type) {
        if (sensors.raw.sensors[type].timer) {
            BaseType_t success;
            success = xTimerDelete(sensors.raw.sensors[type].timer, FAIL_DELAY);
            assert(success == pdTRUE);
        }
    }
    // Task beenden
    vTaskDelete(sensors.taskHandle);
    sensors.taskHandle = NULL;
    // Locks löschen
    vSemaphoreDelete(sensors.raw.lock);
    sensors.raw.lock = NULL;
    vSemaphoreDelete(sensors.state.lock);
    sensors.state.lock = NULL;
    // Struktur der registrierten Sensoren löschen
    memset(&sensors.raw.sensors, 0, sizeof(rawSensor_t) * SENSORS_MAX);
    // Stati löschen
    memset(&sensors.state.data, 0, sizeof(sensorsData_t) * SENSORS_STATE_MAX * SENSORS_ENU_MAX);
}

bool sensorsRegister(sensorsType_t type, sensorsProcessCallback_t callback, void *cookie, TickType_t interval) {
    if (type >= SENSORS_MAX) return true;
    if (sensors.raw.sensors[type].callback && callback) return true;
    // Registrierung
    if (callback) {
        sensors.raw.sensors[type].callback = callback;
        sensors.raw.sensors[type].cookie = cookie;
        if (interval) {
            // Sensor verlangt ein Intervall. Erstelle ein Softwaretimer.
            sensors.raw.sensors[type].timer = xTimerCreate("", interval, pdTRUE, &sensors.raw.sensors[type].timer, &timerCallback);
            if (!sensors.raw.sensors[type].timer) {
                return true;
            }
            // Timer direkt starten falls System bereits aktiv
            if (sensors.taskHandle) {
                BaseType_t success;
                success = xTimerStart(sensors.raw.sensors[type].timer, FAIL_DELAY);
                assert(success == pdTRUE);
            }
        }
    // Deregistrierung
    } else {
        sensors.raw.sensors[type].callback = NULL;
        sensors.raw.sensors[type].cookie = NULL;
        if (sensors.raw.sensors[type].timer) {
            BaseType_t success;
            success = xTimerDelete(sensors.raw.sensors[type].timer, portMAX_DELAY);
            assert(success == pdTRUE);
            sensors.raw.sensors[type].timer = NULL;
        }
    }
    return false;
}

bool sensorsNotify(sensorsType_t type) {
    if (type >= SENSORS_MAX || !sensors.taskHandle || !sensors.raw.sensors[type].callback) return true;
    xTaskNotify(sensors.taskHandle, 0x1 << type, eSetBits);
    return false;
}
bool sensorsNotifyFromISR(sensorsType_t type) {
    if (type >= SENSORS_MAX || !sensors.taskHandle || !sensors.raw.sensors[type].callback) return true;
    BaseType_t woken;
    xTaskNotifyFromISR(sensors.taskHandle, 0x1 << type, eSetBits, &woken);
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    return false;
}

bool sensorsSetRaw(sensorsType_t type, sensorsData_t *data) {
    if (type >= SENSORS_MAX || !data) return true;
    if (xTaskGetCurrentTaskHandle() != sensors.taskHandle) return true;
    sensors.raw.sensors[type].data = *data;
    sensors.raw.sensors[type].needsProcessing = true;
    return false;
}

bool sensorsGetRaw(sensorsType_t type, sensorsData_t *data) {
    if (type >= SENSORS_MAX || !data) return true;
    lockRaw();
    if (sensors.raw.sensors[type].data.timestamp == 0) return true; // nicht aktiv
    *data = sensors.raw.sensors[type].data;
    unlockRaw();
    return false;
}

bool sensorsGetState(sensorsState_t state, sensorsENU_t reference, sensorsData_t *data) {
    if (state >= SENSORS_STATE_MAX || reference >= SENSORS_ENU_MAX || !data) return true;
    // Sensor nicht bereit wenn timestamp noch nie gesetzt wurde
    lockState();
    bool ready = sensors.state.data[state][reference].timestamp > 0;
    if (ready) {
        *data = sensors.state.data[state][reference];
    }
    unlockState();
    return !ready;
}


/**
 * @brief Implementation Privater Funktionen
 * 
 */

static void sensorsTask(void *args) {
    uint32_t toService;
    while (true) {
        xTaskNotifyWait(0x00, ULONG_MAX, &toService, portMAX_DELAY);
        // Jeden anstehenden Sensor der ein Service verlangt abarbeiten.
        lockRaw();
        for (sensorsType_t type = 0; type < SENSORS_MAX; ++type) {
            if (toService & (0x1 << type)) {
                sensors.raw.sensors[type].callback(sensors.raw.sensors[type].cookie);
            }
        }
        // neue Daten verarbeiten
        lockState();
        for (sensorsType_t type = 0; type < SENSORS_MAX; ++type) {
            if (sensors.raw.sensors[type].needsProcessing) {
                processRaw(type);
                if (type >= SENSORS_ACCELERATION && type <= SENSORS_HEIGHT_ABOVE_GROUND) {
                    fuse(type);
                }
                sensors.raw.sensors[type].needsProcessing = false;
            }
        }
        unlockRaw();
        unlockState();
    }
}

static void timerCallback(TimerHandle_t expiredTimer) {
    for (sensorsType_t type = 0; type < SENSORS_MAX; ++type) {
        if (sensors.raw.sensors[type].timer == expiredTimer) {
            sensorsNotify(type);
            return;
        }
    }
    assert(false);
}

static void lockRaw() {
    BaseType_t success;
    success = xSemaphoreTake(sensors.raw.lock, FAIL_DELAY);
    assert(success == pdTRUE);
}

static void unlockRaw() {
    BaseType_t success;
    success = xSemaphoreGive(sensors.raw.lock);
    assert(success == pdTRUE);
}

static void lockState() {
    BaseType_t success;
    success = xSemaphoreTake(sensors.state.lock, FAIL_DELAY);
    assert(success == pdTRUE);
}

static void unlockState() {
    BaseType_t success;
    success = xSemaphoreGive(sensors.state.lock);
    assert(success == pdTRUE);
}

static void processRaw(sensorsType_t type) {
    sensorsData_t *raw = &sensors.raw.sensors[type].data;
    sensorsData_t *local;
    sensorsData_t *world;
    // Rohdaten in beide ENU Repräsentationen rechnen
    switch (type) {
        case (SENSORS_ORIENTATION): {
            local = &sensors.state.data[SENSORS_STATE_ORIENTATION][SENSORS_ENU_LOCAL];
            world = &sensors.state.data[SENSORS_STATE_ORIENTATION][SENSORS_ENU_WORLD];
            processOrientation(raw, local, world);
            sensorsData_t *localEuler = &sensors.state.data[SENSORS_STATE_EULER][SENSORS_ENU_LOCAL];
            localEuler->reference = SENSORS_ENU_LOCAL;
            localEuler->timestamp = raw->timestamp;
            quaternionToEuler(&local->quaternion, &localEuler->vector);
            sensorsData_t *worldEuler = &sensors.state.data[SENSORS_STATE_EULER][SENSORS_ENU_WORLD];
            worldEuler->reference = SENSORS_ENU_WORLD;
            worldEuler->timestamp = raw->timestamp;
            quaternionToEuler(&world->quaternion, &worldEuler->vector);
            break;
        }
        case (SENSORS_ROTATION): {
            local = &sensors.state.data[SENSORS_STATE_ROTATION][SENSORS_ENU_LOCAL];
            world = &sensors.state.data[SENSORS_STATE_ROTATION][SENSORS_ENU_WORLD];
            processVector(raw, local, world);
            break;
        }
        case (SENSORS_ACCELERATION): {
            local = &sensors.state.data[SENSORS_STATE_ACCELERATION][SENSORS_ENU_LOCAL];
            world = &sensors.state.data[SENSORS_STATE_ACCELERATION][SENSORS_ENU_WORLD];
            processVector(raw, local, world);
            break;
        }
        case (SENSORS_OPTICAL_FLOW):
            ensureReference(raw, SENSORS_ENU_WORLD);
            break;
        case (SENSORS_HEIGHT_ABOVE_SEA):
        case (SENSORS_HEIGHT_ABOVE_GROUND):
        case (SENSORS_GROUNDSPEED):
        case (SENSORS_POSITION):
            ensureReference(raw, SENSORS_ENU_WORLD);
            break;
        case (SENSORS_VOLTAGE): {
            local = &sensors.state.data[SENSORS_STATE_VOLTAGE][SENSORS_ENU_LOCAL];
            *local = *raw;
            break;
        }
        default:
            assert(false);
    }
    return;
}

static void processOrientation(sensorsData_t *raw, sensorsData_t *local, sensorsData_t *world) {
    if (raw->reference == SENSORS_ENU_LOCAL) {
        *local = *raw;
        world->reference = SENSORS_ENU_WORLD;
        world->timestamp = raw->timestamp;
        world->quaternion.i = -(raw->quaternion.i);
        world->quaternion.j = -(raw->quaternion.j);
        world->quaternion.k = -(raw->quaternion.k);
        world->quaternion.real = raw->quaternion.real;
    } else {
        *world = *raw;
        local->reference = SENSORS_ENU_LOCAL;
        local->timestamp = raw->timestamp;
        local->quaternion.i = -(raw->quaternion.i);
        local->quaternion.j = -(raw->quaternion.j);
        local->quaternion.k = -(raw->quaternion.k);
        local->quaternion.real = raw->quaternion.real;
    }
}

static void processVector(sensorsData_t *raw, sensorsData_t *local, sensorsData_t *world) {
    if (raw->reference == SENSORS_ENU_LOCAL) {
        *local = *raw;
        *world = *raw;
        world->reference = SENSORS_ENU_WORLD;
        rotateVector(&world->vector, &sensors.state.data[SENSORS_STATE_ORIENTATION][SENSORS_ENU_LOCAL].quaternion);
    } else {
        *world = *raw;
        *local = *raw;
        local->reference = SENSORS_ENU_LOCAL;
        rotateVector(&local->vector, &sensors.state.data[SENSORS_STATE_ORIENTATION][SENSORS_ENU_WORLD].quaternion);
    }
}

static void quaternionToEuler(sensorsQuaternion_t *q, sensorsVector_t *euler) {
    // von: https://github.com/MartinWeigel/Quaternion/blob/master/Quaternion.c
    // Reihenfolge: ZYX
    // Roll (x-axis rotation)
    float sinr_cosp = +2.0f * (q->real * q->i + q->j * q->k);
    float cosr_cosp = +1.0f - 2.0f * (q->i * q->i + q->j * q->j);
    euler->x = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = +2.0f * (q->real * q->j - q->k * q->i);
    if (fabsf(sinp) >= 1.0f) {
        euler->y = copysignf(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    } else {
        euler->y = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = +2.0f * (q->real * q->k + q->i * q->j);
    float cosy_cosp = +1.0f - 2.0f * (q->j * q->j + q->k * q->k);
    euler->z = atan2f(siny_cosp, cosy_cosp);
    return;
}

static void rotateVector(sensorsVector_t *vector, sensorsQuaternion_t *q) {
    // von: https://github.com/MartinWeigel/Quaternion/blob/master/Quaternion.c
    sensorsVector_t result;

    float ww = q->real * q->real;
    float xx = q->i * q->i;
    float yy = q->j * q->j;
    float zz = q->k * q->k;
    float wx = q->real * q->i;
    float wy = q->real * q->j;
    float wz = q->real * q->k;
    float xy = q->i * q->j;
    float xz = q->i * q->k;
    float yz = q->j * q->k;

    // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
    // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
    // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

    result.x = ww*vector->x + 2.0f*wy*vector->z - 2.0f*wz*vector->y +
            xx*vector->x + 2.0f*xy*vector->y + 2.0f*xz*vector->z -
            zz*vector->x - yy*vector->x;
    result.y = 2.0f*xy*vector->x + yy*vector->y + 2.0f*yz*vector->z +
            2.0f*wz*vector->x - zz*vector->y + ww*vector->y -
            2.0f*wx*vector->z - xx*vector->y;
    result.z = 2.0f*xz*vector->x + 2.0f*yz*vector->y + zz*vector->z -
            2.0f*wy*vector->x - yy*vector->z + 2.0f*wx*vector->y -
            xx*vector->z + ww*vector->z;

    // Copy result to output
    *vector = result;

    return;
}

static void ensureReference(sensorsData_t *data, sensorsENU_t reference) {
    if (data->reference != reference) {
        data->reference = reference;
        rotateVector(&data->vector, &sensors.state.data[SENSORS_STATE_ORIENTATION][!reference].quaternion);
    }
}

static void fuse(sensorsType_t type) {
    // ToDo
    return;
}

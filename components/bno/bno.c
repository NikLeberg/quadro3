/**
 * @file bno.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief HAL-Implementation für sh2 Bibliothek von Hillcrest.
 * @version 0.1
 * @date 2020-12-09
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * @note "Glue-Code" zwischen sh2, i2c und sensors-component
 * @note BNO050 Sensor liefert Orientierung, Rotierung, Beschleunigung und Luftdruck. Die Rohdaten werden grob vorverarbeitet
 * und dann an sensor-component weitergeleitet zur Verarbeitung.
 * @note Component läuft in keinem eigenen Task sondern nutzt das Callback-System des sensor-components.
 * 
 */

/**
 * @brief Includes
 * 
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <assert.h>
#include <math.h>

#include "sensors.h"
#include "driver/gpio.h"
#include "i2c.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "bno.h"


/**
 * @brief Typdeklarationen
 * 
 */

#define FAIL_DELAY              (10 / portTICK_PERIOD_MS)
#define BNO_SHTP_HEADER_LENGTH  4

#define BNO_LINEAR_ACCELERATION_STD_DEVIATION   (0.35)      // 0.35 m/s2 gem. Kp 6.7
#define BNO_PRESSURE_STD_DEVIATION              (0.017)     // 1.7 cm
#define BNO_GYROSCOPE_STD_DEVIATION             (0.05411)   // 3.1 °/s gem. Kp 6.7

/**
 * @brief Zustand des Treibers
 * 
 */
typedef enum {
    BNO_STATE_STOPPED,          // System läuft nicht / uninitialisiert
    BNO_STATE_STARTUP,          // System soll starten, warte auf Reset
    BNO_STATE_ENABLE_REPORTS,   // Reports aktivieren
    BNO_STATE_STARTED,          // wurde gestartet und voll funktionstüchtig
    BNO_STATE_STOPPING          // soll stoppen
} bnoState_t;


/**
 * @brief Variablendeklarationen
 * 
 */

static struct {
    SemaphoreHandle_t apiLock;    
    bnoState_t state;

    sh2_Hal_t hal;

    struct {
        int64_t timestamp; // Zeitpunkt des letzten Empfangs-Interrupts
        uint16_t remaining; // Anzahl verbleibender Bytes eines aktuellen Transfers
    } rx;

    struct {
        uint8_t state; // 0 - nicht aktiviert, 1 - wird aktiviert, 2 - ist aktiviert
        sh2_SensorId_t id;
        sh2_SensorConfig_t config;
    } reports[4];
} bno;


/**
 * @brief Private Funktionsprototypen
 * 
 */

/**
 * @brief Sperre öffentliche API
 * 
 */
static void lockApi();

/**
 * @brief Entsperre öffentliche API
 * 
 */
static void unlockApi();

/**
 * @brief Callback für sensors-component für die allgemeine Verwaltung.
 * Von hier aus wird der BNO gestartet, gestoppt und konfigueriert.
 * 
 * @param cookie unbenutzter Cookie
 */
static void process(void *cookie);

/**
 * @brief PIN-Interrupt.
 * Wird durch negative Flanke an Interrupt Pin aufgerufen, Sensor hat Daten zu senden.
 * Fordere sensors-component auf den Hauptprozess aufzurufen.
 * 
 * @param cookie unbenutzter Cookie
 */
static void interrupt(void *cookie);

/**
 * @brief Asynchrone sh2 Events.
 * Bei Sensorreset, aktivierung von Sensoren oder rx/tx-Fehlern wird dieser Callback aufgerufen.
 * 
 * @param cookie unbenutzter Cookie
 * @param pEvent Pointer zu einem Eventobjekt 
 */
static void asyncEvent(void *cookie, sh2_AsyncEvent_t *pEvent);

/**
 * @brief Setze relevante Sensoren mit benötigten Intervallen.
 * 
 */
static void setDefaultSensors();

/**
 * @brief Daten empfangen.
 * 
 * @param cookie unbenutzter Cookie
 * @param pEvent erhaltener Datenevent, muss mit sh2_decodeSensorEvent() dekodiert werden
 */
static void newData(void *cookie, sh2_SensorEvent_t *pEvent);

/**
 * @brief HAL-Funktion open, startet die Verbindung mit dem Sensor und setzt diesen per Resetpin zurück.
 * 
 * @param self ignoriert
 * @return int 0 - erfolgreich, 1 - Fehler
 */
static int halOpen(sh2_Hal_t *self);

/**
 * @brief HAL-Funktion close, deinitialisiert den Sensor.
 * 
 * @param self ignoriert
 */
static void halClose(sh2_Hal_t *self);

/**
 * @brief HAL-Funktion read, lese SHTP-Packet per I2C.
 * 
 * @param self ignoriert
 * @param pBuffer[out] Buffer um eingelesenes Packet abzulegen
 * @param len grösse des Buffers
 * @param t_us[out] muss Zeitpunkt
 * @return int länge des gelesenen Packets, oder 0 wenn nichts gelesen.
 */
static int halRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);

/**
 * @brief HAL-Funktion write, schreibe SHTP-Packet per I2C.
 * 
 * @param self ignoriert
 * @param pBuffer Buffer zu sendender Daten
 * @param len länge der Daten
 * @return int länge der tatsächlich gesendeter Daten, oder 0 wenn nicht gesendet.
 */
static int halWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

/**
 * @brief HAL-Funktion getTimeUs, erhalte Systemzeit in us.
 * 
 * @param self ignoriert
 * @return uint32_t Systemzeit in Microsekunden
 */
static uint32_t halGetTimeUs(sh2_Hal_t *self);


/**
 * @brief Implementation Öffentlicher Funktionen
 * 
 */

bool bnoStart() {
    if (bno.apiLock) return true; // bereits gestartet
    bno.apiLock = xSemaphoreCreateMutex();
    if (!bno.apiLock) return true;
    bno.state = BNO_STATE_STARTUP;
    if (sensorsRegister(SENSORS_ORIENTATION, process, NULL, 0)) return true;
    // Manuell ein Event auslösen damit auch ohne Interrupt vom BNO der Prozess 1x gestartet wird.
    return sensorsNotify(SENSORS_ORIENTATION);
}

bool bnoStop() {
    if (!bno.apiLock) return true; // bereits gestoppt
    lockApi();
    if (bno.state < BNO_STATE_STARTED) {
        unlockApi();
        return true; // Startvorgang kann nicht abgebrochen werden
    }
    bno.state = BNO_STATE_STOPPING;
    unlockApi();
    return sensorsNotify(SENSORS_ORIENTATION); // sofort Event auslösen
}


/**
 * @brief Implementation Privater Funktionen
 * 
 */

static void lockApi() {
    BaseType_t success;
    success = xSemaphoreTake(bno.apiLock, FAIL_DELAY);
    assert(success == pdTRUE);
}

static void unlockApi() {
    BaseType_t success;
    success = xSemaphoreGive(bno.apiLock);
    assert(success == pdTRUE);
}

static void process(void *cookie) {
    (void) cookie;
    lockApi();
    // je nach Zustand
    switch (bno.state) {
        // Gestoppt, wir sollten nicht hier sein
        case (BNO_STATE_STOPPED):
            assert(false);
            break;
        // Start durch bnoStart() angefordert. Initialisiere HAL und starte sh2.
        case (BNO_STATE_STARTUP): {
            bno.hal.open = halOpen;
            bno.hal.close = halClose;
            bno.hal.read = halRead;
            bno.hal.write = halWrite;
            bno.hal.getTimeUs = halGetTimeUs;
            sh2_open(&bno.hal, asyncEvent, NULL);
            // hierhin kommen wir erst, wenn reset fertig
            break;
        }
        // Reset erfolgt, aktiviere nun die relevanten Sensorreports
        case (BNO_STATE_ENABLE_REPORTS): {
            for (uint8_t i = 0; i < 4; ++i) {
                if (bno.reports[i].state == 2) continue; // Sensor bereits aktiviert, gehe zum nächsten
                if (bno.reports[i].state == 1) break; // Sensor wird gerade aktiviert, abwarten
                bno.reports[i].state = 1; // markiere als in Aktivierung
                sh2_setSensorConfig(bno.reports[i].id, &bno.reports[i].config);
                break; // nur ein Report auf einmal aktivieren
            }
            // wenn der letzte Sensor aktiviert wurde ist das System komplett gestartet
            if (bno.reports[3].state == 2) bno.state = BNO_STATE_STARTED;
            break;
        }
        case (BNO_STATE_STARTED):
            // idle
            break;
        // angewiesen per bnoStop() das System zu beenden
        case (BNO_STATE_STOPPING):
            sensorsRegister(SENSORS_ORIENTATION, NULL, NULL, 0);
            sh2_close();
            bno.state = BNO_STATE_STOPPED;
            vSemaphoreDelete(bno.apiLock);
            bno.apiLock = NULL;
            return; // unlockApi() würde sonst mit gelöschtem lock arbeiten
    }
    // System läuft, service den Sensor
    if (bno.state > BNO_STATE_STARTUP) {
        sh2_service();
    }
    unlockApi();
    return;
}

static void interrupt(void *cookie) {
    (void) cookie;
    bno.rx.timestamp = esp_timer_get_time();
    sensorsNotifyFromISR(SENSORS_ORIENTATION);
}

static void asyncEvent(void *cookie, sh2_AsyncEvent_t *pEvent) {
    (void) cookie;
    switch (pEvent->eventId) {
        // Resetvorgang beendet
        case (SH2_RESET):
            sh2_setSensorCallback(newData, NULL);
            setDefaultSensors();
            bno.state = BNO_STATE_ENABLE_REPORTS;
            break;
        // RX/TX Fehler
        case (SH2_SHTP_EVENT):
            // Ignorieren
            break;
        // Sensor aktiviert
        case (SH2_GET_FEATURE_RESP): {
            for (uint8_t i = 0; i < 4; ++i) {
                if (bno.reports[i].id == pEvent->sh2SensorConfigResp.sensorId) {
                    bno.reports[i].state = 2; // markiere als aktiviert
                }
            }
            break;
        }
    }
    return;
}

static void setDefaultSensors() {
    sh2_SensorId_t id[] = {SH2_ROTATION_VECTOR, SH2_LINEAR_ACCELERATION, SH2_PRESSURE, SH2_GYROSCOPE_CALIBRATED};
    uint32_t interval[] = {2500 /*  400 Hz */, 10000 /* 100 Hz */, 100000 /* 10 Hz */, 2500 /* 400 Hz */};
    for (uint32_t i = 0; i < 4; ++i) {
        bno.reports[i].state = 0;
        bno.reports[i].id = id[i];
        bno.reports[i].config.reportInterval_us = interval[i];
    }
    return;
}

static void newData(void *cookie, sh2_SensorEvent_t *pEvent) {
    (void) cookie;
    sh2_SensorValue_t value;
    sh2_decodeSensorEvent(&value, pEvent);
    sensorsData_t data;
    // übersetze sh2_SensorValue_t in sensorsData_t
    data.reference = SENSORS_ENU_LOCAL;
    data.timestamp = value.timestamp;
    switch (value.sensorId) {
        case (SH2_LINEAR_ACCELERATION): {
            data.vector.x = value.un.linearAcceleration.x;
            data.vector.y = value.un.linearAcceleration.y;
            data.vector.z = value.un.linearAcceleration.z;
            sensorsReal_t accuracy = (4.0 - value.status) * BNO_LINEAR_ACCELERATION_STD_DEVIATION; 
            sensorsSetRaw(SENSORS_ACCELERATION, &data, accuracy);
            break;
        }
        case (SH2_ROTATION_VECTOR):
            data.quaternion.i = value.un.rotationVector.i;
            data.quaternion.j = value.un.rotationVector.j;
            data.quaternion.k = value.un.rotationVector.k;
            data.quaternion.real = value.un.rotationVector.real;
            sensorsSetRaw(SENSORS_ORIENTATION, &data, value.un.rotationVector.accuracy);
            break;
        case (SH2_PRESSURE): // Druck in Meter über Meer umrechnen
            data.reference = SENSORS_ENU_WORLD;
            data.vector.x = 0.0;
            data.vector.y = 0.0;
            data.vector.z = -(228.15f / 0.0065f) * (1.0f - powf(value.un.pressure.value / 1013.25f, (1.0f / 5.255f)));
            sensorsSetRaw(SENSORS_HEIGHT_ABOVE_SEA, &data, BNO_PRESSURE_STD_DEVIATION);
            break;
        case (SH2_GYROSCOPE_CALIBRATED):
            data.vector.x = value.un.gyroscope.x;
            data.vector.y = value.un.gyroscope.y;
            data.vector.z = value.un.gyroscope.z;
            sensorsReal_t accuracy = (4.0 - value.status) * BNO_GYROSCOPE_STD_DEVIATION;
            sensorsSetRaw(SENSORS_ROTATION, &data, accuracy);
            break;
        default:
            assert(false);
    }
}

static int halOpen(sh2_Hal_t *self) {
    (void) self;
    // Interrupt aktivieren
    gpio_config_t interruptPin;
    interruptPin.pin_bit_mask = ((1ULL) << BNO_INTERRUPT_PIN);
    interruptPin.mode = GPIO_MODE_INPUT;
    interruptPin.pull_up_en = GPIO_PULLUP_DISABLE;
    interruptPin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    interruptPin.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&interruptPin);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (gpio_isr_handler_add(BNO_INTERRUPT_PIN, interrupt, NULL)) return 1;
    // Sensor-Reset
    gpio_config_t resetPin;
    resetPin.pin_bit_mask = ((1ULL) << BNO_RESET_PIN);
    resetPin.mode = GPIO_MODE_OUTPUT;
    resetPin.pull_up_en = GPIO_PULLUP_DISABLE;
    resetPin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    resetPin.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&resetPin);
    gpio_set_level(BNO_RESET_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(BNO_RESET_PIN, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    return 0;
}

static void halClose(sh2_Hal_t *self) {
    (void) self;
    // Interrupt- und Resetpin zurücksetzen
    gpio_isr_handler_remove(BNO_INTERRUPT_PIN);
    gpio_reset_pin(BNO_INTERRUPT_PIN);
    gpio_reset_pin(BNO_RESET_PIN);
    return;
}

static int halRead(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    (void) self;
    *t_us = bno.rx.timestamp;
    uint16_t cargoLength = 0;
    // bei aktiver fragmentierter Kommunikation entspricht remaining der Bytes die noch nachgeholt werden müssen
    if (bno.rx.remaining == 0) {
        // Header empfangen
        uint8_t header[BNO_SHTP_HEADER_LENGTH];
        if (i2cRead(BNO_I2C_ADDRESS, header, BNO_SHTP_HEADER_LENGTH, portMAX_DELAY)) return 0;
        // Daten mit Länge gemäss SHTP-Header empfangen
        cargoLength = ((header[1] << 8) + (header[0])) & 0x7fff;
        if (cargoLength == 0) return 0; // nichts zu lesen
        bno.rx.remaining = cargoLength;
    }
    uint16_t length = bno.rx.remaining + BNO_SHTP_HEADER_LENGTH;
    if (length > len) length = len;
    if (i2cRead(BNO_I2C_ADDRESS, pBuffer, length, portMAX_DELAY)) return 0;
    bno.rx.remaining -= (length - BNO_SHTP_HEADER_LENGTH);
    // gelesene Länge zurückgeben
    return length;
}

static int halWrite(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    (void) self;
    if (i2cWrite(BNO_I2C_ADDRESS, pBuffer, len, portMAX_DELAY)) {
        return 0;
    } else {
        return len;
    }
}

static uint32_t halGetTimeUs(sh2_Hal_t *self) {
    (void) self;
    return esp_timer_get_time();
}

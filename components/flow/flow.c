/**
 * @file flow.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Mateksys Optical Flow Sensor basierend auf PMW3901.
 * @version 0.1
 * @date 2020-12-16
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * Kommuniziert über Multiwii Serial Protokoll Version 2 (MSPv2 -> https://github.com/iNavFlight/inav/wiki/MSP-V2)
 * - Sendet mit ~10Hz Flow Rate in Pixel
 * - ebenfalls sendet es Lidar Entfernung in mm (max. 2 m)
 * 
 */

/**
 * @brief Includes
 * 
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <assert.h>
#include <math.h>
#include "driver/uart.h"

#include "sensors.h"

#include "flow.h"


/**
 * @brief Typdeklarationen
 * 
 */

typedef struct __attribute__((packed)) {
    uint8_t start;      // $
    uint8_t version;    // X
    uint8_t direction;  // <
    uint8_t flags;      // immer 0
    uint16_t id;        // 0x1f01 = range, 0x1f02 = flow
    uint16_t size;      // Payload länge
} flowMspv2Header_t;

#define FLOW_MOTION_ID  0x1f02
#define FLOW_MOTION_THRESHOLD_USABLE (255 / 2)

typedef struct __attribute__((packed)) {
    uint8_t quality;    // 0 - 255, Optische Qualität
    int32_t x;          // anzahl Pixelbewegungen in X-Achse
    int32_t y;          // anzahl Pixelbewegungen in Y-Achse
} flowMotion_t;

#define FLOW_RANGE_ID  0x1f01
#define FLOW_RANGE_THRESHOLD_USABLE (255)

typedef struct __attribute__((packed)) {
    uint8_t quality;    // 255 -> Gültig, 0 -> Ungültig
    int32_t distance;   // Distanz in mm
} flowRange_t;

#define FLOW_RX_MINIMAL_LENGTH      (sizeof(flowMspv2Header_t) + sizeof(flowRange_t) + 1)    // Header + Range + CRC
#define FLOW_RX_MAXIMAL_LENGTH      (sizeof(flowMspv2Header_t) + sizeof(flowMotion_t) + 1)      // Header + Motion + CRC


/**
 * @brief Variablendeklarationen
 * 
 */

static struct {
    float scaleX, scaleY; // XY-Skalierung des Optischen Flusses
} flow;


/**
 * @brief Private Funktionsprototypen
 * 
 */

/**
 * @brief Callback für sensors-component.
 * 
 * @param cookie unbenutzter Cookie
 */
static void process(void *cookie);

/**
 * @brief Suche Start eines Datenpackets
 * 
 * @param buffer[out] Empfangspuffer
 * @param position[out] Aktuelle Position im Puffer, sollte immer 0 sein. Funktion setzt den Wert auf 1 sobald Start gefunden wurde.
 * @param timestamp Zeitpunkt an dem der Start gefunden wurde.
 */
static void searchStart(uint8_t *buffer, uint16_t *position, int64_t *timestamp);

/**
 * @brief Empfange den Header
 * 
 * @param buffer[in,out] Empfangspuffer
 * @param position[in,out] aktuelle Position im Puffer
 */
static void receiveHeader(uint8_t *buffer, uint16_t *position);

/**
 * @brief Prüfe den empfangenen Header
 * 
 * @param header[in] Pointer auf Header
 * @param position[in,out] aktuelle Position im Empfangspuffer, wird auf 0 gesetzt sollte der Header nicht entsprechen 
 */
static void checkHeader(flowMspv2Header_t *header, uint16_t *position);

/**
 * @brief Empfange Payload gemäss Header-Id
 * 
 * @param buffer Empfangspuffer
 * @param position aktuelle Position im Puffer
 * @param id FLOW_MOTION_ID oder FLOW_RANGE_ID
 */
static void receivePayload(uint8_t *buffer, uint16_t *position, uint16_t id);

/**
 * @brief Prüfe mittels Prüfsumme
 * 
 * @param buffer Empfangsputter
 * @param length Länge der valider Bytes im Puffer
 * @return true - CRC Fehler, false - CRC übereinstimmend
 */
static bool checkCrc(uint8_t *buffer, uint16_t length);


/**
 * @brief Implementation Öffentlicher Funktionen
 * 
 */

bool flowStart() {
    if (uart_is_driver_installed(FLOW_UART)) return true;
    if (uart_driver_install(FLOW_UART, 2 * UART_FIFO_LEN, 0, 0, NULL, 0)) return true;
    uart_config_t uartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    if (uart_param_config(FLOW_UART, &uartConfig)) return true;
    if (uart_set_pin(FLOW_UART, UART_PIN_NO_CHANGE, FLOW_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) return true;
    // Uart Driver gibt keine Möglichkeit nur RX Interrupts abzufangen. Man müsste die gesamte ISR-standart Logik selber
    // implementieren. Nutze daher die Timerfunktion von sensors-component.
    const TickType_t timerInterval = 1000 / 5 / portTICK_PERIOD_MS; // Daten werden mit ~10 Hz gesendet. Prüfe mit 5 Hz.
    if (sensorsRegister(SENSORS_OPTICAL_FLOW, process, NULL, timerInterval)) return true;
    return false;
}

bool flowStop() {
    if (!uart_is_driver_installed(FLOW_UART)) return true;
    sensorsRegister(SENSORS_OPTICAL_FLOW, NULL, NULL, 0);
    // ToDo: Driver darf nicht gelöscht werden solange noch darauf zugegriffen wird.
    if (uart_driver_delete(FLOW_UART)) return true;
    return false;
}


/**
 * @brief Implementation Privater Funktionen
 * 
 */

static void process(void *cookie) {
    (void) cookie;
    static uint16_t position = 0;
    static uint8_t buffer[FLOW_RX_MAXIMAL_LENGTH];
    static int64_t timestamp;
    flowMspv2Header_t *header = (flowMspv2Header_t*) buffer;
    flowMotion_t *motion = (flowMotion_t*) (buffer + sizeof(flowMspv2Header_t));
    flowRange_t *range = (flowRange_t*) (buffer + sizeof(flowMspv2Header_t));

    while (true) {
        // Solange Bytes verfügbar sind
        size_t available;
        uart_get_buffered_data_len(FLOW_UART, &available);
        if (!available) return;

        // Suche nach Header Start
        while (position == 0 && available--) {
            searchStart(buffer, &position, &timestamp);
        }

        // Header
        if (position > 0) {
            // Rest des Headers empfangen
            if (position < sizeof(flowMspv2Header_t)) {
                receiveHeader(buffer, &position);
            }
            // Empfangener Header prüfen
            if (position == sizeof(flowMspv2Header_t)) {
                checkHeader(header, &position);
            }
        }

        // Payload einlesen
        if (position >= sizeof(flowMspv2Header_t)) {
            receivePayload(buffer, &position, header->id);
        }

        // Payload vollständig
        if (position > sizeof(flowMspv2Header_t) && header->size + sizeof(flowMspv2Header_t) == position) {
            // CRC prüfen
            if (!checkCrc(buffer, position)) {
                sensorsData_t data = {0};
                data.timestamp = timestamp;
                data.reference = SENSORS_ENU_LOCAL;
                if (header->id == FLOW_MOTION_ID && motion->quality >= FLOW_MOTION_THRESHOLD_USABLE) {
                    sensorsData_t gyro;
                    if (!sensorsGetState(SENSORS_STATE_ROTATION, SENSORS_ENU_LOCAL, &gyro)) {
                        data.vector.x = motion->x * flow.scaleX; // skaliere von pixel/s auf rad/s
                        data.vector.y = motion->y * flow.scaleY;
                        data.vector.x -= gyro.vector.x; // entferne Eigenrotation
                        data.vector.y -= gyro.vector.y;
                        sensorsSetRaw(SENSORS_OPTICAL_FLOW, &data);
                    }
                } else if (header->id == FLOW_RANGE_ID && range->quality >= FLOW_RANGE_THRESHOLD_USABLE && range->distance >= 0) {
                    data.vector.z = -(range->distance / 1000.0); // mm -> m
                    sensorsSetRaw(SENSORS_HEIGHT_ABOVE_GROUND, &data);
                }
            }
            position = 0;
        }
    }

    return;
}

static void searchStart(uint8_t *buffer, uint16_t *position, int64_t *timestamp) {
    size_t readBytes = uart_read_bytes(FLOW_UART, buffer, 1, 0);
    if (readBytes == 1 && buffer[0] == '$') {
        *position = 1;
        *timestamp = esp_timer_get_time();
    }
}

static void receiveHeader(uint8_t *buffer, uint16_t *position) {
    uint16_t toRead = sizeof(flowMspv2Header_t) - *position;
    size_t readBytes = uart_read_bytes(FLOW_UART, buffer + *position, toRead, 0);
    if (readBytes > 0) {
        *position += readBytes;
    }
}

static void checkHeader(flowMspv2Header_t *header, uint16_t *position) {
    if (header->start != '$' || header->version != 'X' || header->direction != '<' || header->flags != 0
    || (header->id != FLOW_MOTION_ID && header->id != FLOW_RANGE_ID)
    || header->size < sizeof(flowRange_t) || header->size > sizeof(flowMotion_t)) {
        *position = 0;
    }
}

static void receivePayload(uint8_t *buffer, uint16_t *position, uint16_t id) {
    uint16_t readPayload = *position - sizeof(flowMspv2Header_t);
    uint16_t toRead;
    if (id == FLOW_MOTION_ID) toRead = sizeof(flowMotion_t) - readPayload;
    else if (id == FLOW_RANGE_ID) toRead = sizeof(flowRange_t) - readPayload;
    else assert(false); // sollten nicht hier sein
    size_t readBytes = uart_read_bytes(FLOW_UART, buffer + *position, toRead, 0);
    if (readBytes > 0) {
        *position += readBytes;
    }
}

static bool checkCrc(uint8_t *buffer, uint16_t length) {
    uint8_t crcRx;
    if (uart_read_bytes(FLOW_UART, &crcRx, 1, 0) != 1) return true;
    uint8_t crc = 0;
    for (uint16_t i = 3; i < length; ++i) { // ohne "$X<"
        crc ^= buffer[i];
        for (uint16_t j = 0; j < 8; ++j) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc = crc << 1;
        }
    }
    return crc != crcRx;
}

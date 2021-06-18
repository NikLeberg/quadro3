/**
 * @file adder.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Addition von Ganzzahlen (Vorlage)
 * @version 0.3
 * @date 2021-05-26
 * 
 * @copyright Copyright (c) 2021 Leuenberger Niklaus
 * 
 * Dieses Modul dient als Vorlage für andere Module.
 * 
 */


/*
 * Includes
 * 
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#include "adder.h"


/*
 * Typdeklarationen
 * 
 */

/* ... */


/*
 * Variablendeklarationen
 * 
 */

/* ... */


/*
 * Private Funktionsprototypen
 * 
 */

/* ... */


/*
 * Implementation Öffentlicher Funktionen
 * 
 */

int add1(int a, int b) {
    return a + b;
}

int add2(addition_t *addition) {
    if (!addition) {
        return 0;
    }
    addition->result = addition->a + addition->b;
    return addition->result;
}


/*
 * Implementation Privater Funktionen
 * 
 */

/* ... */

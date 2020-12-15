/* Example test application for testable component.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_system.h"
#include <stdio.h>
#include <string.h>
#include "unity.h"

void app_main(void) {
#ifdef CI_TEST_IN_QEMU
    printf("\nRunning all ci registered tests.");
    printf("\n-----------------------\n\n");
    UNITY_BEGIN();
    unity_run_tests_by_tag("[ci]", false);
    UNITY_END();
    esp_restart();
#else
    printf("\nRunning all the registered tests.");
    printf("\n-----------------------\n\n");
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
#endif
}

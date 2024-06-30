/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "iot_button.h"

#define BUTTON_IO_NUM 0
#define BUTTON_ACTIVE_LEVEL 0

#define TAG "BOARD"

uint8_t mode = 0;
extern void change_sta_to_ap();

static void button_tap_cb(void* arg) {
    ESP_LOGI(TAG, "tap cb (%s)", (char*)arg);
    if (mode == 0) {
        mode = 1;
        change_sta_to_ap();
    }
}

static void board_button_init(void) {
    button_handle_t btn_handle =
        iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb,
                              "RELEASE");
    }
}

void board_init(void) { board_button_init(); }

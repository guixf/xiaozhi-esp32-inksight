#include "ssd1683_display.h"
#include <esp_log.h>
#include <string.h>
#include <cstdio>
#include <esp_http_client.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <esp_tls.h>
#include <esp_sntp.h>
#include <sys/time.h>
#include <time.h>

static const char* TAG = "Ssd1683Display";

static const uint8_t LUT_VCOM_DC[] = {
    0x00, 0x17, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x00, 0x0A, 0x01, 0x00, 0x00, 0x01,
    0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t LUT_WW[] = {
    0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
    0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
    0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t LUT_BW[] = {
    0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
    0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
    0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t LUT_WB[] = {
    0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
    0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
    0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t LUT_BB[] = {
    0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
    0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
    0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define ROBOT_ICON_W 24
#define ROBOT_ICON_H 24
static const uint8_t ROBOT_ICON[72] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x7E, 0x00,
    0x00, 0x7E, 0x00, 0x07, 0xFF, 0xE0, 0x0F, 0xFF, 0xF0, 0x0C, 0x00, 0x30,
    0x0C, 0x00, 0x30, 0x3C, 0x00, 0x3C, 0x7C, 0xC3, 0x3E, 0x7C, 0xC3, 0x3E,
    0x7C, 0xC3, 0x3E, 0x7C, 0x00, 0x3E, 0x3C, 0x00, 0x3C, 0x0C, 0xFF, 0x30,
    0x0C, 0xFF, 0x30, 0x0C, 0x00, 0x30, 0x0C, 0x00, 0x30, 0x0F, 0xFF, 0xF0,
    0x07, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

Ssd1683Display::Ssd1683Display(int width, int height, ssd1683_spi_t spi_data)
    : width_(width), height_(height), spi_data_(spi_data),
      spi_handle_(nullptr), spi_mutex_(nullptr),
      img_buf_(nullptr), refresh_count_(0),
      display_state_(DisplayState::IDLE), voice_state_(VoiceState::INACTIVE),
      voice_icon_x_(-1), voice_icon_y_(0),
      inksight_fetch_pending_(false),
      display_busy_(false), display_pending_(false), spi_initialized_(false), pending_image_(nullptr) {
    spi_mutex_ = xSemaphoreCreateMutex();
}

Ssd1683Display::~Ssd1683Display() {
    if (img_buf_) {
        free(img_buf_);
    }
    if (spi_handle_) {
        spi_bus_remove_device(spi_handle_);
    }
    vSemaphoreDelete(spi_mutex_);
}

void Ssd1683Display::EPD_Init() {
    if (!img_buf_) {
        img_buf_ = (uint8_t*)heap_caps_malloc(spi_data_.buffer_len, MALLOC_CAP_SPIRAM);
        if (!img_buf_) {
            ESP_LOGE(TAG, "Failed to allocate image buffer");
            return;
        }
        memset(img_buf_, 0xFF, spi_data_.buffer_len);

        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = spi_data_.mosi;
        buscfg.miso_io_num = -1;
        buscfg.sclk_io_num = spi_data_.scl;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.data4_io_num = -1;
        buscfg.data5_io_num = -1;
        buscfg.data6_io_num = -1;
        buscfg.data7_io_num = -1;

        ESP_ERROR_CHECK(spi_bus_initialize(spi_data_.spi_host, &buscfg, SPI_DMA_CH_AUTO));

        spi_device_interface_config_t devcfg = {};
        devcfg.command_bits = 0;
        devcfg.address_bits = 0;
        devcfg.dummy_bits = 0;
        devcfg.mode = 0;
        devcfg.duty_cycle_pos = 0;
        devcfg.cs_ena_pretrans = 0;
        devcfg.cs_ena_posttrans = 0;
        devcfg.clock_speed_hz = SPI_MASTER_FREQ_40M;
        devcfg.input_delay_ns = 0;
        devcfg.spics_io_num = spi_data_.cs;
        devcfg.flags = 0;
        devcfg.queue_size = 1;

        ESP_ERROR_CHECK(spi_bus_add_device(spi_data_.spi_host, &devcfg, &spi_handle_));

        gpio_set_direction((gpio_num_t)spi_data_.dc, GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)spi_data_.rst, GPIO_MODE_OUTPUT);
        gpio_set_direction((gpio_num_t)spi_data_.busy, GPIO_MODE_INPUT);
    }

    Reset();

    EPD_SendCommand(0x12);
    read_busy();

    EPD_SendCommand(0x21);
    EPD_SendData(0x40);
    EPD_SendData(0x00);

    EPD_SendCommand(0x3C);
    EPD_SendData(0x05);

 //   EPD_SendCommand(0x11);
 //   EPD_SendData(0x03);

    EPD_SendCommand(0x44);
    EPD_SendData(0);
    EPD_SendData((width_ / 8 - 1) & 0xFF);
    EPD_SendCommand(0x45);
    EPD_SendData(0);
    EPD_SendData(0);
    EPD_SendData((height_ - 1) & 0xFF);
    EPD_SendData(((height_ - 1) >> 8) & 0xFF);

    EPD_SendCommand(0x4E);
    EPD_SendData(0);
    EPD_SendCommand(0x4F);
    EPD_SendData(0);
    EPD_SendData(0);

    EPD_SendCommand(0x24);
    for (int i = 0; i < spi_data_.buffer_len; i++) {
        EPD_SendData(0xFF);
    }

    EPD_SendCommand(0x22);
    EPD_SendData(0xF7);
    EPD_SendCommand(0x20);
    read_busy();

    ESP_LOGI(TAG, "EPD initialized (full mode)");
}

void Ssd1683Display::EPD_InitFast() {
    Reset();
    read_busy();

    EPD_SendCommand(0x12);
    read_busy();

    EPD_SendCommand(0x21);
    EPD_SendData(0x40);
    EPD_SendData(0x00);

    EPD_SendCommand(0x3C);
    EPD_SendData(0x05);

    EPD_SendCommand(0x1A);
    EPD_SendData(0x6E);

    EPD_SendCommand(0x22);
    EPD_SendData(0x91);
    EPD_SendCommand(0x20);
    read_busy();

    EPD_SendCommand(0x11);
    EPD_SendData(0x03);

    EPD_SendCommand(0x44);
    EPD_SendData(0);
    EPD_SendData((width_ / 8 - 1) & 0xFF);
    EPD_SendCommand(0x45);
    EPD_SendData(0);
    EPD_SendData(0);
    EPD_SendData((height_ - 1) & 0xFF);
    EPD_SendData(((height_ - 1) >> 8) & 0xFF);

    EPD_SendCommand(0x4E);
    EPD_SendData(0);
    EPD_SendCommand(0x4F);
    EPD_SendData(0);
    EPD_SendData(0);

    ESP_LOGI(TAG, "EPD initialized (fast mode)");
}

void Ssd1683Display::Reset() {
    gpio_set_level((gpio_num_t)spi_data_.rst, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)spi_data_.rst, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Ssd1683Display::EPD_SendCommand(uint8_t command) {
    if (xSemaphoreTake(spi_mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }
    gpio_set_level((gpio_num_t)spi_data_.dc, 0);
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &command;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &t));
    xSemaphoreGive(spi_mutex_);
}

void Ssd1683Display::EPD_SendData(uint8_t data) {
    if (xSemaphoreTake(spi_mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }
    gpio_set_level((gpio_num_t)spi_data_.dc, 1);
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &data;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &t));
    xSemaphoreGive(spi_mutex_);
}

void Ssd1683Display::EPD_SendDataArray(const uint8_t* data, int len) {
    if (xSemaphoreTake(spi_mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }
    gpio_set_level((gpio_num_t)spi_data_.dc, 1);
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &t));
    xSemaphoreGive(spi_mutex_);
}

void Ssd1683Display::writeBytes(const uint8_t* data, int length) {
    EPD_SendDataArray(data, length);
}

void Ssd1683Display::readBytes(uint8_t* data, int length) {
    if (xSemaphoreTake(spi_mutex_, portMAX_DELAY) != pdTRUE) {
        return;
    }
    gpio_set_level((gpio_num_t)spi_data_.dc, 1);
    spi_transaction_t t = {};
    t.length = length * 8;
    t.rx_buffer = data;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &t));
    xSemaphoreGive(spi_mutex_);
}

void Ssd1683Display::read_busy() {
    int busy_count = 0;
    while (gpio_get_level((gpio_num_t)spi_data_.busy) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        busy_count++;
        if (busy_count > 500) {
            ESP_LOGW(TAG, "Busy timeout");
            break;
        }
    }
}

void Ssd1683Display::EPD_SetFullWindow(int x_start, int y_start, int x_end, int y_end) {
    EPD_SendCommand(0x44);
    EPD_SendData(x_start / 8);
    EPD_SendData(x_end / 8);
    EPD_SendCommand(0x45);
    EPD_SendData(y_start & 0xFF);
    EPD_SendData((y_start >> 8) & 0xFF);
    EPD_SendData(y_end & 0xFF);
    EPD_SendData((y_end >> 8) & 0xFF);
}

void Ssd1683Display::EPD_Clear() {
    if (!img_buf_) {
        return;
    }
    memset(img_buf_, 0xFF, spi_data_.buffer_len);
    EPD_Display(img_buf_);
}


void Ssd1683Display::EPD_Display(const uint8_t* image) {
    if (!img_buf_ || display_busy_) {
        return;
    }

    display_busy_ = true;
    display_state_ = DisplayState::DISPLAYING;

    EPD_Init();

    EPD_SendCommand(0x4E);
    EPD_SendData(0);
    EPD_SendCommand(0x4F);
    EPD_SendData(0);
    EPD_SendData(0);

    EPD_SendCommand(0x24);
    for (int i = 0; i < spi_data_.buffer_len; i++) {
        EPD_SendData(image[i]);
    }

    EPD_SendCommand(0x26);
    for (int i = 0; i < spi_data_.buffer_len; i++) {
        EPD_SendData(image[i]);
    }

    EPD_SendCommand(0x22);
    EPD_SendData(0xF7);
    EPD_SendCommand(0x20);

    read_busy();
    display_busy_ = false;
    display_state_ = DisplayState::IDLE;

    refresh_count_++;
    ESP_LOGI(TAG, "Display updated (full, cycle %d)", refresh_count_);
}


void Ssd1683Display::EPD_DisplayFast(const uint8_t* image) {
    if (!img_buf_ || display_busy_) {
        return;
    }

    display_busy_ = true;
    display_state_ = DisplayState::DISPLAYING;

    EPD_InitFast();

    EPD_SendCommand(0x4E);
    EPD_SendData(0);
    EPD_SendCommand(0x4F);
    EPD_SendData(0);
    EPD_SendData(0);

    EPD_SendCommand(0x24);
    for (int i = 0; i < spi_data_.buffer_len; i++) {
        EPD_SendData(image[i]);
    }

    if (red_buf_) {
        int red_pixels = 0;
        int total_bytes = spi_data_.buffer_len;
        for (int i = 0; i < total_bytes; i++) {
            if (red_buf_[i] != 0xFF) {
                red_pixels += __builtin_popcount(~red_buf_[i]);
            }
        }
        ESP_LOGI(TAG, "[DISPLAY RED DEBUG FAST] red_buf_ size: %d bytes, red pixels count: %d", total_bytes, red_pixels);
        writeBytes(red_buf_, spi_data_.buffer_len);
    } else {
        ESP_LOGI(TAG, "[DISPLAY RED DEBUG FAST] red_buf_ is NULL, using image instead");
        writeBytes(image, spi_data_.buffer_len);
    }

    EPD_SendCommand(0x22);
    EPD_SendData(0xC7);
    EPD_SendCommand(0x20);

    read_busy();
    display_busy_ = false;
    display_state_ = DisplayState::IDLE;

    refresh_count_++;
    ESP_LOGI(TAG, "Display updated (fast, cycle %d)", refresh_count_);
}

void Ssd1683Display::EPD_DisplayPartial(const uint8_t* image, int x_start, int y_start, int x_end, int y_end) {
    if (!img_buf_ || display_busy_) {
        return;
    }

    display_busy_ = true;
    display_state_ = DisplayState::PARTIAL_UPDATING;

    int x_start_byte = x_start / 8;
    int x_end_byte = (x_end - 1) / 8;

    EPD_SendCommand(0x44);
    EPD_SendData(x_start_byte);
    EPD_SendData(x_end_byte);
    EPD_SendCommand(0x45);
    EPD_SendData(y_start & 0xFF);
    EPD_SendData((y_start >> 8) & 0xFF);
    EPD_SendData((y_end - 1) & 0xFF);
    EPD_SendData(((y_end - 1) >> 8) & 0xFF);

    EPD_SendCommand(0x4E);
    EPD_SendData(x_start_byte);

    EPD_SendCommand(0x4F);
    EPD_SendData(y_start & 0xFF);
    EPD_SendData((y_start >> 8) & 0xFF);

    EPD_SendCommand(0x24);
    int width_bytes = x_end_byte - x_start_byte + 1;
    int height = y_end - y_start;
    for (int row = 0; row < height; row++) {
        writeBytes(image + row * width_bytes, width_bytes);
    }

    EPD_SendCommand(0x22);
    EPD_SendData(0xCC);
    EPD_SendCommand(0x20);

    read_busy();
    display_busy_ = false;
    display_state_ = DisplayState::IDLE;

    ESP_LOGI(TAG, "Display updated (partial)");
}

void Ssd1683Display::EPD_Sleep() {
    EPD_SendCommand(0x10);
    EPD_SendData(0x01);
    ESP_LOGI(TAG, "EPD entered sleep mode");
}

void Ssd1683Display::clearBuffer() {
    if (img_buf_) {
        memset(img_buf_, 0xFF, spi_data_.buffer_len);
    }
}

/*void Ssd1683Display::RequestDisplayUpdate() {
    if (display_busy_) {
        display_pending_ = true;
        if (pending_image_) {
            free(pending_image_);
        }
        pending_image_ = (uint8_t*)malloc(spi_data_.buffer_len);
        if (pending_image_) {
            memcpy(pending_image_, img_buf_, spi_data_.buffer_len);
        }
        ESP_LOGI(TAG, "Display update pending");
    } else {
        EPD_DisplayFast(img_buf_);
    }
}

*/

void Ssd1683Display::RequestDisplayUpdate() {
    if (display_state_ == DisplayState::IDLE) {
        // 三色屏必须始终使用完整刷新，快刷会导致红色显示异常
        // 因为快刷模式 (0xC7) 不会正确更新红色通道
        bool use_fast_refresh = false;  // 强制使用完整刷新
        
        if (red_buf_) {
            ESP_LOGI(TAG, "[DISPLAY] Color display detected, forcing full refresh");
        }
        if (use_fast_refresh) {
            ESP_LOGI(TAG, "[DISPLAY] Always active mode, using fast refresh");
            EPD_DisplayFast(img_buf_);
        } else {
            ESP_LOGI(TAG, "[DISPLAY] Using full refresh");
            EPD_Display(img_buf_);  // 使用完整刷新，正确显示红色
        }
    } else {
        display_pending_ = true;
        if (pending_image_) {
            free(pending_image_);
        }
        pending_image_ = (uint8_t*)heap_caps_malloc(spi_data_.buffer_len, MALLOC_CAP_SPIRAM);
        memcpy(pending_image_, img_buf_, spi_data_.buffer_len);
    }
}
/*
void Ssd1683Display::ProcessDisplayUpdate() {
    if (display_busy_ && gpio_get_level((gpio_num_t)spi_data_.busy) == 1) {
        return;
    }

    if (display_busy_) {
        display_busy_ = false;
        display_state_ = DisplayState::IDLE;

        if (display_pending_ && pending_image_) {
            EPD_DisplayFast(pending_image_);
            free(pending_image_);
            pending_image_ = nullptr;
            display_pending_ = false;
        }
    }
}
*/

void Ssd1683Display::ProcessDisplayUpdate() {
    switch (display_state_) {
        case DisplayState::IDLE:
            if (display_pending_ && pending_image_) {
                // 三色屏必须始终使用完整刷新
                bool use_fast_refresh = false;  // 强制使用完整刷新
                if (use_fast_refresh) {
                    ESP_LOGI(TAG, "[DISPLAY] Always active mode, using fast refresh");
                    EPD_DisplayFast(pending_image_);
                } else {
                    ESP_LOGI(TAG, "[DISPLAY] Using full refresh");
                    EPD_Display(pending_image_);
                }
                display_state_ = DisplayState::DISPLAYING;
            }
            break;

        case DisplayState::DISPLAYING:
        case DisplayState::PARTIAL_UPDATING:
            if (gpio_get_level((gpio_num_t)spi_data_.busy) == 0) {
                display_state_ = DisplayState::IDLE;
                if (pending_image_) {
                    free(pending_image_);
                    pending_image_ = nullptr;
                }
                display_pending_ = false;
            }
            break;

        default:
            break;
    }
}



bool Ssd1683Display::IsDisplayBusy() {
    return display_busy_;
}

void Ssd1683Display::smartDisplay(const uint8_t* image) {
    if (!img_buf_ || display_busy_) {
        return;
    }

    display_busy_ = true;
    display_state_ = DisplayState::DISPLAYING;

    EPD_SendCommand(0x4E);
    EPD_SendData(0);
    EPD_SendCommand(0x4F);
    EPD_SendData(0);
    EPD_SendData(0);

    EPD_SendCommand(0x24);
    for (int i = 0; i < spi_data_.buffer_len; i++) {
        EPD_SendData(image[i]);
    }
    
    EPD_SendCommand(0x26);//刷写红色
    for (int i = 0; i < spi_data_.buffer_len; i++) {
        EPD_SendData(image[i]);
    }
    
    if (refresh_count_ % 10 == 0) {
        EPD_SendCommand(0x22);
        EPD_SendData(0xF7);
        ESP_LOGI(TAG, "smartDisplay: full refresh (cycle %d)", refresh_count_);
    } else {
        EPD_SendCommand(0x22);
        EPD_SendData(0xC7);
        ESP_LOGI(TAG, "smartDisplay: fast refresh (cycle %d)", refresh_count_);
    }
    EPD_SendCommand(0x20);

    refresh_count_++;
}

void Ssd1683Display::TriggerImageUpdate() {
    if (external_image_callback_ && img_buf_) {
        external_image_callback_(img_buf_);
    }
}

void Ssd1683Display::SetStatus(const char* status) {
    ESP_LOGW(TAG, "SetStatus: %s", status);
    
    // 当进入待命状态时，获取 inksight 图片
    if (status && (
        strcmp(status, "待命") == 0 || 
        strcmp(status, "standby") == 0 || 
        strcmp(status, "STANDBY") == 0 ||
        strcmp(status, "Idle") == 0 ||
        strcmp(status, "idle") == 0 ||
        strcmp(status, "IDLE") == 0
    )) {
        ESP_LOGI(TAG, "[INKSIGHT] Standby state detected, fetching image...");
        // 获取 Inksight 图片
        FetchInksightImage();
        // 发送心跳
        SendHeartbeat(true);
    }
}

void Ssd1683Display::drawText(const char* msg, int x, int y, int scale) {
    if (!msg || !img_buf_) return;
    int rowBytes = width_ / 8;
    int len = strlen(msg);

    for (int ci = 0; ci < len; ci++) {
        const uint8_t* glyph = getGlyph(msg[ci]);
        int cx = x + ci * (5 * scale + scale);
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (glyph[col] & (1 << row)) {
                    for (int dy = 0; dy < scale; dy++) {
                        for (int dx = 0; dx < scale; dx++) {
                            int px = cx + col * scale + dx;
                            int py = y + row * scale + dy;
                            if (px >= 0 && px < width_ && py >= 0 && py < height_)
                                img_buf_[py * rowBytes + px / 8] &= ~(0x80 >> (px % 8));
                        }
                    }
                }
            }
        }
    }
}

void Ssd1683Display::drawPixel(int x, int y, bool black) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_ || !img_buf_) return;
    int rowBytes = width_ / 8;
    if (black) {
        img_buf_[y * rowBytes + x / 8] &= ~(0x80 >> (x % 8));
    } else {
        img_buf_[y * rowBytes + x / 8] |= (0x80 >> (x % 8));
    }
}

void Ssd1683Display::fillRect(int x, int y, int w, int h) {
    if (!img_buf_) return;
    int rowBytes = width_ / 8;
    for (int py = y; py < y + h; py++) {
        if (py < 0 || py >= height_) continue;
        for (int px = x; px < x + w; px++) {
            if (px < 0 || px >= width_) continue;
            img_buf_[py * rowBytes + px / 8] &= ~(0x80 >> (px % 8));
        }
    }
}

int Ssd1683Display::textWidth(int charCount, int scale) {
    return charCount * (5 * scale + scale) - scale;
}

const uint8_t* Ssd1683Display::getGlyph(char c) {
    static const uint8_t g_space[] = {0x00, 0x00, 0x00, 0x00, 0x00};

    static const uint8_t g_A[] = {0x7E,0x11,0x11,0x11,0x7E};
    static const uint8_t g_B[] = {0x7F,0x49,0x49,0x49,0x36};
    static const uint8_t g_C[] = {0x3E,0x41,0x41,0x41,0x22};
    static const uint8_t g_D[] = {0x7F,0x41,0x41,0x22,0x1C};
    static const uint8_t g_E[] = {0x7F,0x49,0x49,0x49,0x41};
    static const uint8_t g_F[] = {0x7F,0x09,0x09,0x09,0x01};
    static const uint8_t g_G[] = {0x3E,0x41,0x49,0x49,0x3A};
    static const uint8_t g_H[] = {0x7F,0x08,0x08,0x08,0x7F};
    static const uint8_t g_I[] = {0x00,0x41,0x7F,0x41,0x00};
    static const uint8_t g_J[] = {0x20,0x40,0x41,0x3F,0x01};
    static const uint8_t g_K[] = {0x7F,0x08,0x14,0x22,0x41};
    static const uint8_t g_L[] = {0x7F,0x40,0x40,0x40,0x40};
    static const uint8_t g_M[] = {0x7F,0x02,0x0C,0x02,0x7F};
    static const uint8_t g_N[] = {0x7F,0x04,0x08,0x10,0x7F};
    static const uint8_t g_O[] = {0x3E,0x41,0x41,0x41,0x3E};
    static const uint8_t g_P[] = {0x7F,0x09,0x09,0x09,0x06};
    static const uint8_t g_Q[] = {0x3E,0x41,0x51,0x21,0x5E};
    static const uint8_t g_R[] = {0x7F,0x09,0x19,0x29,0x46};
    static const uint8_t g_S[] = {0x26,0x49,0x49,0x49,0x32};
    static const uint8_t g_T[] = {0x01,0x01,0x7F,0x01,0x01};
    static const uint8_t g_U[] = {0x3F,0x40,0x40,0x40,0x3F};
    static const uint8_t g_V[] = {0x1F,0x20,0x40,0x20,0x1F};
    static const uint8_t g_W[] = {0x3F,0x40,0x38,0x40,0x3F};
    static const uint8_t g_X[] = {0x63,0x14,0x08,0x14,0x63};
    static const uint8_t g_Y[] = {0x07,0x08,0x70,0x08,0x07};
    static const uint8_t g_Z[] = {0x61,0x51,0x49,0x45,0x43};

    static const uint8_t g_a[] = {0x20,0x54,0x54,0x54,0x78};
    static const uint8_t g_b[] = {0x7F,0x48,0x44,0x44,0x38};
    static const uint8_t g_c[] = {0x38,0x44,0x44,0x44,0x28};
    static const uint8_t g_d[] = {0x38,0x44,0x44,0x28,0x7F};
    static const uint8_t g_e[] = {0x38,0x54,0x54,0x54,0x18};
    static const uint8_t g_f[] = {0x00,0x08,0x7E,0x09,0x02};
    static const uint8_t g_g[] = {0x18,0xA4,0xA4,0xA4,0x7C};
    static const uint8_t g_h[] = {0x7F,0x08,0x04,0x04,0x78};
    static const uint8_t g_i[] = {0x00,0x44,0x7D,0x40,0x00};
    static const uint8_t g_k[] = {0x7F,0x10,0x28,0x44,0x00};
    static const uint8_t g_l[] = {0x00,0x41,0x7F,0x40,0x00};
    static const uint8_t g_m[] = {0x7C,0x04,0x18,0x04,0x78};
    static const uint8_t g_n[] = {0x7C,0x08,0x04,0x04,0x78};
    static const uint8_t g_o[] = {0x38,0x44,0x44,0x44,0x38};
    static const uint8_t g_p[] = {0x7C,0x14,0x14,0x14,0x08};
    static const uint8_t g_q[] = {0x38,0x44,0x44,0x34,0x08};
    static const uint8_t g_r[] = {0x7C,0x08,0x04,0x04,0x08};
    static const uint8_t g_s[] = {0x48,0x54,0x54,0x54,0x24};
    static const uint8_t g_t[] = {0x04,0x3F,0x44,0x40,0x20};
    static const uint8_t g_u[] = {0x3C,0x40,0x40,0x20,0x7C};
    static const uint8_t g_v[] = {0x1C,0x20,0x40,0x20,0x1C};
    static const uint8_t g_w[] = {0x3C,0x40,0x30,0x40,0x3C};
    static const uint8_t g_x[] = {0x44,0x28,0x10,0x28,0x44};
    static const uint8_t g_y[] = {0x18,0xA4,0xA4,0xA4,0x7C};
    static const uint8_t g_z[] = {0x44,0x64,0x54,0x4C,0x44};

    static const uint8_t g_0[] = {0x3E,0x51,0x49,0x45,0x3E};
    static const uint8_t g_1[] = {0x00,0x42,0x7F,0x40,0x00};
    static const uint8_t g_2[] = {0x42,0x61,0x51,0x49,0x46};
    static const uint8_t g_3[] = {0x21,0x41,0x45,0x4B,0x31};
    static const uint8_t g_4[] = {0x18,0x14,0x12,0x7F,0x10};
    static const uint8_t g_5[] = {0x27,0x45,0x45,0x45,0x39};
    static const uint8_t g_6[] = {0x3C,0x4A,0x49,0x49,0x30};
    static const uint8_t g_7[] = {0x01,0x71,0x09,0x05,0x03};
    static const uint8_t g_8[] = {0x36,0x49,0x49,0x49,0x36};
    static const uint8_t g_9[] = {0x06,0x49,0x49,0x29,0x1E};

    static const uint8_t g_colon[] = {0x00,0x00,0x36,0x36,0x00};
    static const uint8_t g_dash[] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t g_dot[] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t g_slash[] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t g_exclam[] = {0x00,0x00,0x5F,0x00,0x00};

    switch (c) {
        case 'A': return g_A; case 'B': return g_B; case 'C': return g_C;
        case 'D': return g_D; case 'E': return g_E; case 'F': return g_F;
        case 'G': return g_G; case 'H': return g_H; case 'I': return g_I;
        case 'J': return g_J; case 'K': return g_K; case 'L': return g_L;
        case 'M': return g_M; case 'N': return g_N; case 'O': return g_O;
        case 'P': return g_P; case 'Q': return g_Q; case 'R': return g_R;
        case 'S': return g_S; case 'T': return g_T; case 'U': return g_U;
        case 'V': return g_V; case 'W': return g_W; case 'X': return g_X;
        case 'Y': return g_Y; case 'Z': return g_Z;
        case 'a': return g_a; case 'b': return g_b; case 'c': return g_c;
        case 'd': return g_d; case 'e': return g_e; case 'f': return g_f;
        case 'g': return g_g; case 'h': return g_h; case 'i': return g_i;
        case 'j': return g_J; case 'k': return g_k; case 'l': return g_l;
        case 'm': return g_m; case 'n': return g_n; case 'o': return g_o;
        case 'p': return g_p; case 'q': return g_q; case 'r': return g_r;
        case 's': return g_s; case 't': return g_t; case 'u': return g_u;
        case 'v': return g_v; case 'w': return g_w; case 'x': return g_x;
        case 'y': return g_y; case 'z': return g_z;
        case '0': return g_0; case '1': return g_1; case '2': return g_2;
        case '3': return g_3; case '4': return g_4; case '5': return g_5;
        case '6': return g_6; case '7': return g_7; case '8': return g_8;
        case '9': return g_9;
        case ':': return g_colon; case '-': return g_dash;
        case '.': return g_dot; case '/': return g_slash;
        case '!': return g_exclam; case ' ': return g_space;
        default: return g_space;
    }
}

const char* Ssd1683Display::aiStatusTitle(const char* state) {
    if (!state) return "AI";
    if (strcmp(state, "IDLE") == 0) return "AI IDLE";
    if (strcmp(state, "CONNECTING") == 0) return "CONNECT";
    if (strcmp(state, "LISTENING") == 0) return "LISTEN";
    if (strcmp(state, "THINKING") == 0) return "THINK";
    if (strcmp(state, "SPEAKING") == 0) return "SPEAK";
    return "AI";
}

const char* Ssd1683Display::aiStatusDetailFallback(const char* state) {
    if (!state) return "NON-ASCII";
    if (strcmp(state, "ERROR") == 0) return "ERROR";
    if (strcmp(state, "IDLE") == 0) return "WAITING";
    if (strcmp(state, "CONNECTING") == 0) return "WS CONNECT";
    if (strcmp(state, "LISTENING") == 0) return "MIC LISTEN";
    if (strcmp(state, "THINKING") == 0) return "WORKING";
    if (strcmp(state, "SPEAKING") == 0) return "PLAYING";
    return "NON-ASCII";
}

void Ssd1683Display::drawHLine(int x0, int x1, int y) {
    if (x1 <= x0) return;
    fillRect(x0, y, x1 - x0, 1);
}

void Ssd1683Display::drawWrappedAsciiText(const char* msg, int x, int y, int width, int scale, int maxLines) {
    if (!msg || !msg[0] || width <= 0 || scale <= 0 || maxLines <= 0) return;

    const int charW = 5 * scale + scale;
    int maxChars = width / charW;
    if (maxChars < 1) maxChars = 1;

    const int lineHeight = 7 * scale + scale * 2;
    char lineBuf[96];

    int lineNo = 0;
    int lineLen = 0;

    for (const char* p = msg; *p && lineNo < maxLines; ++p) {
        char ch = *p;
        if (ch == '\n' || ch == '\r') {
            if (lineLen > 0) {
                lineBuf[lineLen] = '\0';
                drawText(lineBuf, x, y + lineNo * lineHeight, scale);
                lineNo++;
                lineLen = 0;
            }
            continue;
        }

        lineBuf[lineLen++] = ch;
        if (lineLen >= maxChars) {
            lineBuf[lineLen] = '\0';
            drawText(lineBuf, x, y + lineNo * lineHeight, scale);
            lineNo++;
            lineLen = 0;
        }
    }

    if (lineLen > 0 && lineNo < maxLines) {
        lineBuf[lineLen] = '\0';
        drawText(lineBuf, x, y + lineNo * lineHeight, scale);
    }
}

int Ssd1683Display::computeMaxWrappedLines(int startY, int endY, int scale) {
    if (endY <= startY) return 0;
    const int textHeight = 7 * scale;
    const int lineHeight = 7 * scale + scale * 2;
    int availableForFirstLine = (endY - startY) - textHeight;
    if (availableForFirstLine < 0) return 0;
    return availableForFirstLine / lineHeight + 1;
}

void Ssd1683Display::drawRobotIcon(int ox, int oy, int scale) {
    int iconRowBytes = ROBOT_ICON_W / 8;
    int rowBytes = width_ / 8;
    for (int row = 0; row < ROBOT_ICON_H; row++) {
        for (int col = 0; col < ROBOT_ICON_W; col++) {
            int iconByte = row * iconRowBytes + col / 8;
            int iconBit = 7 - (col % 8);
            if (!((ROBOT_ICON[iconByte] >> iconBit) & 1)) continue;
            for (int sy = 0; sy < scale; sy++) {
                for (int sx = 0; sx < scale; sx++) {
                    int px = ox + col * scale + sx;
                    int py = oy + row * scale + sy;
                    if (px < 0 || px >= width_ || py < 0 || py >= height_) continue;
                    img_buf_[py * rowBytes + px / 8] &= ~(0x80 >> (px % 8));
                }
            }
        }
    }
}

void Ssd1683Display::showSetupScreen(const char* apName, const char* regCode) {
    clearBuffer();

    int marginX = width_ * 7 / 100;
    fillRect(0, 0, width_, height_ * 12 / 100);
    fillRect(marginX, height_ * 28 / 100, width_ * 84 / 100, height_ * 2 / 100);
    fillRect(marginX, height_ * 72 / 100, width_ * 84 / 100, height_ * 2 / 100);

    const char* title = "SETUP WIFI";
    int titleScale = (height_ < 200) ? 2 : 4;
    int titleX = (width_ - textWidth(strlen(title), titleScale)) / 2;
    int titleY = height_ * 15 / 100;
    drawText(title, titleX, titleY, titleScale);

    const char* line1 = "CONNECT TO";
    int bodyScale = (height_ < 200) ? 1 : 2;
    int line1X = (width_ - textWidth(strlen(line1), bodyScale)) / 2;
    int line1Y = height_ * 36 / 100;
    drawText(line1, line1X, line1Y, bodyScale);

    int apScale = (height_ < 200) ? 2 : 3;
    int apX = (width_ - textWidth(strlen(apName), apScale)) / 2;
    int apY = height_ * 46 / 100;
    drawText(apName, apX, apY, apScale);

    const char* line3 = "OPEN BROWSER";
    int line3X = (width_ - textWidth(strlen(line3), bodyScale)) / 2;
    int line3Y = height_ * 62 / 100;
    drawText(line3, line3X, line3Y, bodyScale);

    if (regCode && regCode[0]) {
        const char* regLabel = "REG CODE:";
        drawText(regLabel, marginX, height_ * 78 / 100, bodyScale);
        drawText(regCode, marginX, height_ * 85 / 100, bodyScale);
    }

    RequestDisplayUpdate();
    ESP_LOGI(TAG, "Setup screen shown: %s", apName);
}

void Ssd1683Display::showAiChatStatus(const char* state, const char* detail) {
    clearBuffer();

    int titleScale = (height_ < 200) ? 2 : 4;
    int bodyScale = (height_ < 200) ? 1 : 2;
    int marginX = width_ * 7 / 100;
    int contentWidth = width_ - marginX * 2;
    if (contentWidth < 40) contentWidth = width_ - 8;

    const char* title = aiStatusTitle(state);
    const char* body = detail;

    int titleWidth = textWidth(strlen(title), titleScale);
    int titleX = (width_ - titleWidth) / 2;
    if (titleX < marginX) titleX = marginX;
    int titleY = height_ * 12 / 100;

    drawText(title, titleX, titleY, titleScale);

    int lineY = titleY + 7 * titleScale + titleScale * 3;
    drawHLine(marginX, width_ - marginX, lineY);
    drawHLine(marginX, width_ - marginX, lineY + 3);

    int bodyY = lineY + ((height_ < 200) ? 12 : 20);
    int yEnd = height_ - ((height_ < 200) ? 10 : 8);
    int maxLines = computeMaxWrappedLines(bodyY, yEnd, bodyScale);

    if (body && body[0]) {
        drawWrappedAsciiText(body, marginX, bodyY, contentWidth, bodyScale, maxLines);
    } else {
        drawWrappedAsciiText(aiStatusDetailFallback(state), marginX, bodyY, contentWidth, bodyScale, maxLines);
    }

    RequestDisplayUpdate();
    ESP_LOGI(TAG, "AI chat status: %s", state);
}

void Ssd1683Display::showVoiceChatScreen() {
    clearBuffer();

    int scale = (height_ < 200) ? 2 : 4;
    int iconDrawW = ROBOT_ICON_W * scale;
    int iconDrawH = ROBOT_ICON_H * scale;
    int ix = (width_ - iconDrawW) / 2;
    int iy = (height_ - iconDrawH) / 2 - ((height_ < 200) ? 8 : 20);
    drawRobotIcon(ix, iy, scale);

    int labelScale = (height_ < 200) ? 1 : 2;
    const char* label = "AI Chat";
    int labelW = textWidth(strlen(label), labelScale);
    int labelX = (width_ - labelW) / 2;
    int labelY = iy + iconDrawH + ((height_ < 200) ? 6 : 16);
    drawText(label, labelX, labelY, labelScale);

    RequestDisplayUpdate();
    ESP_LOGI(TAG, "[VOICE] chat screen shown");
}

void Ssd1683Display::showVoiceIndicator(bool footerCenter) {
    if (!img_buf_) return;

    int iconW = ROBOT_ICON_W;
    int iconH = ROBOT_ICON_H;
    int padding = 4;
    int ix = footerCenter ? ((width_ - iconW) / 2) : (width_ - iconW - padding);
    int iy = height_ - iconH - padding;
    if (ix < 0) ix = 0;
    if (iy < 0) iy = 0;
    voice_icon_x_ = ix;
    voice_icon_y_ = iy;

    int iconRowBytes = iconW / 8;
    int rowBytes = width_ / 8;
    for (int row = 0; row < iconH; row++) {
        int srcByteIdx = (iy + row) * rowBytes + (ix / 8);
        memcpy(voice_icon_backup_ + row * iconRowBytes, img_buf_ + srcByteIdx, iconRowBytes);
    }

    drawRobotIcon(ix, iy, 1);

    int xStart = (ix / 8) * 8;
    int xEnd = xStart + iconW + 8;
    if (xEnd > width_) xEnd = width_;
    int yStart = iy;
    int yEnd = iy + iconH;
    if (yEnd > height_) yEnd = height_;

    int xS = xStart / 8;
    int xE = (xEnd - 1) / 8;
    int widthBytes = xE - xS + 1;
    int height = yEnd - yStart;

    for (int row = 0; row < height; row++) {
        memcpy(
            voice_partial_buf_ + row * widthBytes,
            img_buf_ + (yStart + row) * rowBytes + xS,
            widthBytes
        );
    }

    voice_state_ = VoiceState::ACTIVE;
    EPD_DisplayPartial(voice_partial_buf_, xStart, yStart, xEnd, yEnd);
    ESP_LOGI(TAG, "[VOICE] indicator shown");
}

void Ssd1683Display::hideVoiceIndicator() {
    if (voice_icon_x_ < 0 || voice_icon_y_ < 0 || !img_buf_) return;

    int iconW = ROBOT_ICON_W;
    int iconH = ROBOT_ICON_H;
    int iconRowBytes = iconW / 8;
    int rowBytes = width_ / 8;

    for (int row = 0; row < iconH; row++) {
        int dstByteIdx = (voice_icon_y_ + row) * rowBytes + (voice_icon_x_ / 8);
        memcpy(img_buf_ + dstByteIdx, voice_icon_backup_ + row * iconRowBytes, iconRowBytes);
    }

    int xStart = (voice_icon_x_ / 8) * 8;
    int xEnd = xStart + iconW + 8;
    if (xEnd > width_) xEnd = width_;
    int yStart = voice_icon_y_;
    int yEnd = voice_icon_y_ + iconH;
    if (yEnd > height_) yEnd = height_;

    int xS = xStart / 8;
    int xE = (xEnd - 1) / 8;
    int widthBytes = xE - xS + 1;
    int height = yEnd - yStart;

    for (int row = 0; row < height; row++) {
        memcpy(
            voice_partial_buf_ + row * widthBytes,
            img_buf_ + (yStart + row) * rowBytes + xS,
            widthBytes
        );
    }

    voice_state_ = VoiceState::INACTIVE;
    EPD_DisplayPartial(voice_partial_buf_, xStart, yStart, xEnd, yEnd);
    voice_icon_x_ = -1;
    voice_icon_y_ = -1;
    ESP_LOGI(TAG, "[VOICE] indicator hidden");
}

void Ssd1683Display::showError(const char* msg) {
    clearBuffer();

    int scale = (height_ < 200) ? 1 : 2;
    int len = strlen(msg);
    int startX = (width_ - textWidth(len, scale)) / 2;
    int startY = height_ / 2 - (7 * scale) / 2;

    drawText(msg, startX, startY, scale);

    RequestDisplayUpdate();
    ESP_LOGI(TAG, "Error shown: %s", msg);
}

void Ssd1683Display::showDiagnostic(const char* line1, const char* line2, const char* line3, const char* line4) {
    clearBuffer();

    int titleScale = (height_ < 200) ? 2 : 3;
    int bodyScale = (height_ < 200) ? 1 : 2;

    int y = height_ * 10 / 100;
    int lh = 7 * titleScale + titleScale * 2;
    int blh = 7 * bodyScale + bodyScale * 2;

    if (line1 && line1[0]) {
        int x = (width_ - textWidth(strlen(line1), titleScale)) / 2;
        if (x < 4) x = 4;
        drawText(line1, x, y, titleScale);
        y += lh + 4;
    }

    int infoX = width_ * 5 / 100;
    if (line2 && line2[0]) {
        drawText(line2, infoX, y, bodyScale);
        y += blh;
    }
    if (line3 && line3[0]) {
        drawText(line3, infoX, y, bodyScale);
        y += blh;
    }
    if (line4 && line4[0]) {
        drawText(line4, infoX, y, bodyScale);
    }

    RequestDisplayUpdate();
}

void Ssd1683Display::showModePreview(const char* modeName) {
    clearBuffer();

    int nameScale = (height_ < 200) ? 2 : 3;
    int loadScale = (height_ < 200) ? 1 : 2;

    int len = strlen(modeName);
    int nameX = (width_ - textWidth(len, nameScale)) / 2;
    int nameY = height_ / 2 - (height_ < 200 ? 15 : 30);
    drawText(modeName, nameX, nameY, nameScale);

    const char* loading = "loading...";
    int loadLen = strlen(loading);
    int loadX = (width_ - textWidth(loadLen, loadScale)) / 2;
    int loadY = height_ / 2 + (height_ < 200 ? 10 : 20);
    drawText(loading, loadX, loadY, loadScale);

    RequestDisplayUpdate();
    ESP_LOGI(TAG, "Mode preview shown: %s", modeName);
}

// ── Inksight Image Fetching ─────────────────────────────────────────────

static const char INKIGHT_ROOT_CA[] = R"(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----)";

static bool sntp_sync_done_ = false;

static void sync_ntp_time() {
    if (sntp_sync_done_) {
        return;
    }
    
    ESP_LOGI(TAG, "[INKSIGHT] Syncing time via SNTP...");
    
    esp_sntp_stop();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();
    
    int retry = 0;
    const int max_retry = 10;
    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && retry < max_retry) {
        ESP_LOGI(TAG, "[INKSIGHT] Waiting for NTP sync... (%d/%d)", retry + 1, max_retry);
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
    }
    
    if (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
        sntp_sync_done_ = true;
    } else {
        ESP_LOGW(TAG, "[INKSIGHT] NTP sync failed, setting default time for TLS");
        time_t now = 1735689600;
        struct timeval tv = {now, 0};
        settimeofday(&tv, NULL);
    }
    
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "[INKSIGHT] Current time: %04d-%02d-%02d %02d:%02d:%02d",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

std::string Ssd1683Display::GetMacAddress() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(mac_str);
}

void Ssd1683Display::SaveDeviceToken(const char* token) {
    if (!token || strlen(token) == 0) {
        ClearDeviceToken();
        return;
    }
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_str(nvs_handle, "device_token", token);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        inksight_device_token_ = token;
        ESP_LOGI(TAG, "[INKIGHT] Token saved: %s", token);
    }
}

void Ssd1683Display::ClearDeviceToken() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_key(nvs_handle, "device_token");
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    inksight_device_token_.clear();
    ESP_LOGI(TAG, "[INKIGHT] Token cleared");
}

std::string Ssd1683Display::LoadDeviceToken() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return "";
    }
    char token_str[256];
    size_t len = sizeof(token_str);
    err = nvs_get_str(nvs_handle, "device_token", token_str, &len);
    nvs_close(nvs_handle);
    if (err == ESP_OK) {
        return std::string(token_str);
    }
    return "";
}

bool Ssd1683Display::EnsureDeviceToken() {
    if (!inksight_device_token_.empty()) {
        return true;
    }
    
    // 同步时间，TLS 证书验证需要正确的时间
    sync_ntp_time();
    
    inksight_device_token_ = LoadDeviceToken();
    if (!inksight_device_token_.empty()) {
        ESP_LOGI(TAG, "[INKIGHT] Loaded token from storage");
        return true;
    }

    ESP_LOGI(TAG, "[INKIGHT] Need to fetch new token from server");
    
    // 检查 WiFi 连接状态
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "[INKIGHT] WiFi connected to: %s, RSSI: %d", ap_info.ssid, ap_info.rssi);
    } else {
        ESP_LOGE(TAG, "[INKIGHT] WiFi not connected!");
        return false;
    }
    
    // 获取 IP 地址
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info) == ESP_OK) {
        char ip_str[16];
        inet_ntoa_r(ip_info.ip.addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "[INKIGHT] IP address: %s", ip_str);
    }
    
    std::string mac = GetMacAddress();
    char url[512];
    snprintf(url, sizeof(url), "%s/api/device/%s/token", inksight_server_.c_str(), mac.c_str());
    ESP_LOGI(TAG, "[INKIGHT] Fetch token from: %s", url);

    vTaskDelay(pdMS_TO_TICKS(1200));

    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 15000;
    config.cert_pem = INKIGHT_ROOT_CA;
    config.skip_cert_common_name_check = true;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "[INKIGHT] HTTP client init failed");
        return false;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    bool success = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        esp_err_t err = esp_http_client_open(client, 2);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "[INKIGHT] Failed to open connection: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(800));
            continue;
        }

        int written = esp_http_client_write(client, "{}", 2);
        if (written < 0) {
            ESP_LOGW(TAG, "[INKIGHT] Failed to write POST data");
            esp_http_client_close(client);
            vTaskDelay(pdMS_TO_TICKS(800));
            continue;
        }

        int content_len = esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "[INKIGHT] Token fetch attempt %d, status: %d, len: %d", attempt + 1, status_code, content_len);

        if (status_code >= 200 && status_code < 300 && content_len > 0 && content_len < 8192) {
            char* buffer = (char*)malloc(content_len + 1);
            if (buffer) {
                int total_read = 0;
                int read_len;
                while (total_read < content_len) {
                    read_len = esp_http_client_read(client, buffer + total_read, content_len - total_read);
                    if (read_len <= 0) break;
                    total_read += read_len;
                }
                buffer[total_read] = '\0';
                
                ESP_LOGI(TAG, "[INKIGHT] Response (%d bytes): %s", total_read, buffer);
                
                const char* token_start = strstr(buffer, "\"token\"");
                if (token_start) {
                    token_start = strchr(token_start, ':');
                    if (token_start) {
                        token_start = strchr(token_start, '\"');
                        if (token_start) {
                            token_start++;
                            const char* token_end = strchr(token_start, '\"');
                            if (token_end && token_end > token_start) {
                                std::string token(token_start, token_end - token_start);
                                if (!token.empty()) {
                                    SaveDeviceToken(token.c_str());
                                    success = true;
                                }
                            }
                        }
                    }
                }
                free(buffer);
            }
        }
        esp_http_client_close(client);
        if (success) break;
        vTaskDelay(pdMS_TO_TICKS(800));
    }

    esp_http_client_cleanup(client);
    return success;
}

void Ssd1683Display::SetInksightConfig(const char* server) {
    inksight_server_ = server ? server : "";
    inksight_device_token_ = LoadDeviceToken();
    ESP_LOGI(TAG, "[INKIGHT] Config set: server=%s, token=%s", 
             inksight_server_.c_str(), 
             inksight_device_token_.empty() ? "(empty)" : "(present)");
}

void Ssd1683Display::FetchInksightImage() {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[INKIGHT] Server not configured, skip fetch");
        return;
    }
    if (inksight_fetch_pending_) {
        ESP_LOGI(TAG, "[INKIGHT] Fetch already in progress");
        return;
    }
    inksight_fetch_pending_ = true;
    BaseType_t result = xTaskCreatePinnedToCore(&Ssd1683Display::InksightFetchTask, "inksight_fetch", 12288, this, 5, &inksight_task_handle_, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "[INKIGHT] Failed to create fetch task");
        inksight_fetch_pending_ = false;
        return;
    }
    ESP_LOGI(TAG, "[INKIGHT] Fetch task created");
}

void Ssd1683Display::InksightFetchTask(void* param) {
    Ssd1683Display* self = static_cast<Ssd1683Display*>(param);
    ESP_LOGI(TAG, "[INKIGHT] Fetch task started");
    self->DoFetchInksightImage();
    ESP_LOGI(TAG, "[INKIGHT] Fetch task completed");
    self->inksight_fetch_pending_ = false;
    vTaskDelete(NULL);
}

void Ssd1683Display::DoFetchInksightImage() {
    if (inksight_server_.empty() || !img_buf_) {
        ESP_LOGW(TAG, "[INKIGHT] Skip fetch: server empty or buffer null");
        return;
    }

    if (!EnsureDeviceToken()) {
        ESP_LOGE(TAG, "[INKIGHT] Failed to ensure device token, cannot fetch image");
        return;
    }

    for (int attempt = 0; attempt < 2; attempt++) {
        char url[512];
        snprintf(url, sizeof(url), "%s/api/render?w=%d&h=%d&bpp=1&colors=2",
                  inksight_server_.c_str(), width_, height_);

        ESP_LOGI(TAG, "[INKIGHT] Fetching: %s (attempt %d)", url, attempt + 1);

        esp_http_client_config_t config = {};
        config.url = url;
        config.timeout_ms = 15000;
        config.cert_pem = INKIGHT_ROOT_CA;
        config.skip_cert_common_name_check = true;

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (!client) {
            ESP_LOGE(TAG, "[INKIGHT] HTTP client init failed");
            return;
        }

        if (!inksight_device_token_.empty()) {
            esp_http_client_set_header(client, "X-Device-Token", inksight_device_token_.c_str());
        }
        esp_http_client_set_header(client, "Accept-Encoding", "identity");

        esp_err_t err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "[INKIGHT] Failed to open connection: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int content_len = esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "[INKIGHT] HTTP status: %d, content_len: %d", status_code, content_len);

        if (status_code < 200 || status_code >= 300) {
            ESP_LOGW(TAG, "[INKIGHT] HTTP request failed: %d", status_code);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            
            if (status_code == 401 && attempt == 0) {
                ESP_LOGW(TAG, "[INKIGHT] 401 Unauthorized, clearing token and retrying");
                ClearDeviceToken();
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            return;
        }

        memset(img_buf_, 0xFF, spi_data_.buffer_len);

        // Read BMP file header
        uint8_t fileHeader[14];
        int n = esp_http_client_read(client, (char*)fileHeader, 14);
        if (n != 14) {
            ESP_LOGW(TAG, "[INKIGHT] Failed to read BMP file header");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return;
        }

        // Read pixel data offset from file header (offset 10)
        uint32_t pixelOffset = fileHeader[10]
                            | ((uint32_t)fileHeader[11] << 8)
                            | ((uint32_t)fileHeader[12] << 16)
                            | ((uint32_t)fileHeader[13] << 24);
        ESP_LOGI(TAG, "[INKIGHT] BMP pixel offset: %u", pixelOffset);

        // Read info header to get bit count (not used, but needed to skip)
        uint8_t infoHeader[16];
        n = esp_http_client_read(client, (char*)infoHeader, 16);
        if (n != 16) {
            ESP_LOGW(TAG, "[INKIGHT] Failed to read BMP info header");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return;
        }

        // Skip remaining header and palette
        int toSkip = pixelOffset - 14 - 16;
        while (toSkip > 0) {
            uint8_t skipByte;
            n = esp_http_client_read(client, (char*)&skipByte, 1);
            if (n <= 0) break;
            toSkip--;
        }

        // Calculate row stride (4-byte aligned)
        int rowBytes = width_ / 8;
        int rowStride = (rowBytes + 3) & ~3;

        // Allocate row buffer with correct stride
        uint8_t* rowBuf = (uint8_t*)malloc(rowStride);
        if (!rowBuf) {
            ESP_LOGE(TAG, "[INKIGHT] rowBuf alloc failed");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return;
        }

        int height = height_;
        for (int y = 0; y < height; y++) {
            int dispY = height - 1 - y;
            n = esp_http_client_read(client, (char*)rowBuf, rowStride);
            if (n != rowStride) {
                ESP_LOGW(TAG, "[INKIGHT] Read row %d failed: %d", y, n);
                break;
            }
            memcpy(img_buf_ + dispY * rowBytes, rowBuf, rowBytes);
        }

        free(rowBuf);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);

        ESP_LOGI(TAG, "[INKIGHT] Image fetched, triggering display");
        
ESP_LOGI(TAG, "[INKSIGHT] 2BPP OK %d bytes, converting to display format...", color_buf_len);

int buf_len = spi_data_.buffer_len;
int w = width_ / 8;




memset(img_buf_, 0xFF, buf_len);
memset(red_buf_, 0xFF, buf_len);

for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
        int index = (y * width_ + x) / 4;
        int bitOffset = ((y * width_ + x) % 4) * 2;
        uint8_t color = (color_buf[index] >> (6 - bitOffset)) & 0x03;
        
        int bufIndex = y * w + (x / 8);
        int bitPos = x % 8;
        
        // SSD1683 BWR 三色屏颜色映射:
        // 黑色: 0x24 RAM = 0 (bit cleared), 0x26 RAM = 1 (bit set)
        // 白色: 0x24 RAM = 1 (bit set),   0x26 RAM = 1 (bit set)
        // 红色: 0x24 RAM = 1 (bit set),   0x26 RAM = 0 (bit cleared)
        if (color == 0x00) {
            // 黑色像素
            img_buf_[bufIndex] &= ~(0x80 >> bitPos);
            red_buf_[bufIndex] |= (0x80 >> bitPos);
        } else if (color == 0x01) {
            // 白色像素
            img_buf_[bufIndex] |= (0x80 >> bitPos);
            red_buf_[bufIndex] |= (0x80 >> bitPos);
        } else if (color == 0x02) {
            // 红色像素
            img_buf_[bufIndex] |= (0x80 >> bitPos);
            red_buf_[bufIndex] &= ~(0x80 >> bitPos);
        } else if (color == 0x03) {
            // 保留/深色 - 按黑色处理
            img_buf_[bufIndex] &= ~(0x80 >> bitPos);
            red_buf_[bufIndex] &= ~(0x80 >> bitPos);
        }
    }
}

ESP_LOGI(TAG, "[INKSIGHT] 2BPP conversion complete, red_buf_ initialized");        
        
        
        RequestDisplayUpdate();
        break; // Success, exit loop
    }
}

// ── Heartbeat Functions ────────────────────────────────────────────────

bool Ssd1683Display::IsWifiConnected() {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) return false;
    return ap_info.rssi != 0;
}

void Ssd1683Display::SendHeartbeat(bool force) {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[HEARTBEAT] Server not configured, skip");
        return;
    }
    
    xTaskCreatePinnedToCore(&Ssd1683Display::HeartbeatTask, "heartbeat", 8192, this, 5, NULL, 0);
}

void Ssd1683Display::HeartbeatTask(void* param) {
    Ssd1683Display* self = static_cast<Ssd1683Display*>(param);
    self->DoSendHeartbeat(false);
    vTaskDelete(NULL);
}

void Ssd1683Display::DoSendHeartbeat(bool force) {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[HEARTBEAT] Server not configured");
        return;
    }

    if (!IsWifiConnected()) {
        ESP_LOGW(TAG, "[HEARTBEAT] WiFi not connected");
        return;
    }

    uint64_t now = esp_timer_get_time() / 1000;
    if (!force && last_heartbeat_ms_ != 0 && (now - last_heartbeat_ms_) < HEARTBEAT_INTERVAL_MS) {
        ESP_LOGD(TAG, "[HEARTBEAT] Skip, interval not reached");
        return;
    }

    if (!EnsureDeviceToken()) {
        ESP_LOGE(TAG, "[HEARTBEAT] Failed to ensure device token");
        return;
    }

    std::string mac = GetMacAddress();
    char url[512];
    snprintf(url, sizeof(url), "%s/api/device/%s/heartbeat", inksight_server_.c_str(), mac.c_str());

    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 10000;
    config.cert_pem = INKIGHT_ROOT_CA;
    config.skip_cert_common_name_check = true;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "[HEARTBEAT] HTTP client init failed");
        return;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    if (!inksight_device_token_.empty()) {
        esp_http_client_set_header(client, "X-Device-Token", inksight_device_token_.c_str());
    }

    bool success = false;
    for (int attempt = 0; attempt < 2; attempt++) {
        esp_err_t err = esp_http_client_open(client, 2);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "[HEARTBEAT] Failed to open connection: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int written = esp_http_client_write(client, "{}", 2);
        if (written < 0) {
            ESP_LOGW(TAG, "[HEARTBEAT] Failed to write POST data");
            esp_http_client_close(client);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);

        ESP_LOGI(TAG, "[HEARTBEAT] POST -> %d", status_code);

        if (status_code >= 200 && status_code < 300) {
            last_heartbeat_ms_ = now;
            success = true;
            esp_http_client_close(client);
            break;
        }

        if (status_code == 401 && attempt == 0) {
            ESP_LOGW(TAG, "[HEARTBEAT] 401 Unauthorized, clearing token");
            esp_http_client_close(client);
            ClearDeviceToken();
            if (EnsureDeviceToken()) {
                esp_http_client_set_header(client, "X-Device-Token", inksight_device_token_.c_str());
                continue;
            }
        }
        esp_http_client_close(client);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    esp_http_client_cleanup(client);

    if (success) {
        ESP_LOGI(TAG, "[HEARTBEAT] Success");
    } else {
        ESP_LOGE(TAG, "[HEARTBEAT] Failed");
    }
}

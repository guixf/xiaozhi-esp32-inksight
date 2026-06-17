#include "ssd1683_display.h"
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

#define TAG "SSD1683Display"
#define FULL_REFRESH_INTERVAL 10

static const uint8_t g_lut_vcom_dc[] = {
    0x00, 0x17, 0x00, 0x00, 0x00, 0x02, 0x00, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x00, 0x0A, 0x01, 0x00, 0x00, 0x01, 0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t g_lut_ww[] = {
    0x40, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x40, 0x0A, 0x01, 0x00, 0x00, 0x01, 0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t g_lut_bw[] = {
    0x40, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x40, 0x0A, 0x01, 0x00, 0x00, 0x01, 0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t g_lut_wb[] = {
    0x80, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x80, 0x0A, 0x01, 0x00, 0x00, 0x01, 0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t g_lut_bb[] = {
    0x80, 0x17, 0x00, 0x00, 0x00, 0x02, 0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
    0x80, 0x0A, 0x01, 0x00, 0x00, 0x01, 0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

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
static const uint8_t g_j[] = {0x20,0x40,0x44,0x3D,0x00};
static const uint8_t g_k[] = {0x7F,0x10,0x28,0x44,0x00};
static const uint8_t g_l[] = {0x00,0x41,0x7F,0x40,0x00};
static const uint8_t g_m[] = {0x7C,0x04,0x18,0x04,0x7C};
static const uint8_t g_n[] = {0x7C,0x08,0x04,0x04,0x7C};
static const uint8_t g_o[] = {0x38,0x44,0x44,0x44,0x38};
static const uint8_t g_p[] = {0x7C,0x14,0x14,0x14,0x08};
static const uint8_t g_q[] = {0x38,0x44,0x44,0x44,0x38};
static const uint8_t g_r[] = {0x7C,0x08,0x04,0x04,0x08};
static const uint8_t g_s[] = {0x48,0x54,0x54,0x54,0x24};
static const uint8_t g_t[] = {0x04,0x3F,0x44,0x40,0x20};
static const uint8_t g_u[] = {0x3C,0x40,0x40,0x20,0x7C};
static const uint8_t g_v[] = {0x1C,0x20,0x40,0x20,0x1C};
static const uint8_t g_w[] = {0x3C,0x40,0x30,0x40,0x3C};
static const uint8_t g_x[] = {0x6C,0x10,0x08,0x10,0x6C};
static const uint8_t g_y[] = {0x0C,0x50,0x50,0x50,0x3C};
static const uint8_t g_z[] = {0x64,0x54,0x54,0x4C,0x00};

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
static const uint8_t g_dash[]  = {0x08,0x08,0x08,0x08,0x08};
static const uint8_t g_dot[]   = {0x00,0x60,0x60,0x00,0x00};
static const uint8_t g_slash[] = {0x20,0x10,0x08,0x04,0x02};
static const uint8_t g_exclam[] = {0x00,0x00,0x5F,0x00,0x00};
static const uint8_t g_space[] = {0x00,0x00,0x00,0x00,0x00};

Ssd1683Display::Ssd1683Display(int width, int height, ssd1683_spi_t spi_data)
    : spi_data_(spi_data), width_(width), height_(height), 
      refresh_count_(0), display_pending_(false), display_state_(DisplayState::IDLE),
      pending_image_(nullptr), use_full_refresh_(false) {
    img_buf_ = (uint8_t*)heap_caps_malloc(spi_data.buffer_len, MALLOC_CAP_SPIRAM);
    assert(img_buf_);
    clearBuffer();
    spi_gpio_init();
    spi_port_init();
    EPD_Init();
}

Ssd1683Display::~Ssd1683Display() {
    if (img_buf_) {
        heap_caps_free(img_buf_);
    }
    if (pending_image_) {
        free(pending_image_);
    }
}

bool Ssd1683Display::Lock(int timeout_ms) {
    return true;
}

void Ssd1683Display::Unlock() {
}

void Ssd1683Display::SetEmotion(const char* emotion) {
}

void Ssd1683Display::SetStatus(const char* status) {
    clearBuffer();
    drawText(status, 10, height_ / 2, 2);
    RequestDisplayUpdate();
}

void Ssd1683Display::SetChatMessage(const char* role, const char* content) {
    clearBuffer();
    int x = 10;
    int y = 20;
    drawText(role, x, y, 2);
    y += 30;
    drawText(content, x, y, 1);
    RequestDisplayUpdate();
}

void Ssd1683Display::ShowNotification(const char* notification, int duration_ms) {
    clearBuffer();
    drawText(notification, 10, height_ / 2, 2);
    RequestDisplayUpdate();
}

void Ssd1683Display::UpdateStatusBar(bool update_all) {
}

void Ssd1683Display::SetPowerSaveMode(bool on) {
    if (on) {
        EPD_Sleep();
    } else {
        EPD_Init();
    }
}

void Ssd1683Display::SetTheme(Theme* theme) {
}

void Ssd1683Display::RequestDisplayUpdate() {
    if (display_state_ == DisplayState::IDLE) {
        use_full_refresh_ = (refresh_count_ % FULL_REFRESH_INTERVAL == 0);
        display_state_ = DisplayState::SENDING_DATA;
    } else {
        display_pending_ = true;
        if (pending_image_) {
            free(pending_image_);
        }
        pending_image_ = (uint8_t*)heap_caps_malloc(spi_data_.buffer_len, MALLOC_CAP_SPIRAM);
        memcpy(pending_image_, img_buf_, spi_data_.buffer_len);
    }
}

void Ssd1683Display::ProcessDisplayUpdate() {
    const uint8_t* image = img_buf_;
    
    switch (display_state_) {
        case DisplayState::IDLE:
            if (display_pending_ && pending_image_) {
                use_full_refresh_ = (refresh_count_ % FULL_REFRESH_INTERVAL == 0);
                image = pending_image_;
                display_state_ = DisplayState::SENDING_DATA;
            }
            break;
            
        case DisplayState::SENDING_DATA: {
            EPD_SendCommand(0x24);
            writeBytes(image, spi_data_.buffer_len);
            
            EPD_SendCommand(0x26);
            writeBytes(image, spi_data_.buffer_len);
            
            if (use_full_refresh_) {
                EPD_SendCommand(0x22);
                EPD_SendData(0xF7);
                EPD_SendCommand(0x20);
                refresh_count_ = 0;
            } else {
                EPD_SendCommand(0x22);
                EPD_SendData(0xC7);
                EPD_SendCommand(0x20);
            }
            refresh_count_++;
            
            display_state_ = DisplayState::WAITING_FOR_BUSY;
            break;
        }
            
        case DisplayState::WAITING_FOR_BUSY:
            if (gpio_get_level((gpio_num_t)spi_data_.busy) == 0) {
                display_state_ = DisplayState::COMPLETED;
            }
            break;
            
        case DisplayState::COMPLETED:
            if (pending_image_) {
                free(pending_image_);
                pending_image_ = nullptr;
            }
            display_pending_ = false;
            display_state_ = DisplayState::IDLE;
            break;
    }
}

bool Ssd1683Display::IsDisplayBusy() {
    return display_state_ != DisplayState::IDLE;
}

const uint8_t* Ssd1683Display::getGlyph(char c) {
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
        case 'j': return g_j; case 'k': return g_k; case 'l': return g_l;
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
        case '.': return g_dot;   case '/': return g_slash;
        case '!': return g_exclam;
        default:  return g_space;
    }
}

void Ssd1683Display::drawText(const char* msg, int x, int y, int scale) {
    int row_bytes = width_ / 8;
    int len = strlen(msg);

    for (int i = 0; i < len; i++) {
        char c = msg[i];
        const uint8_t* glyph = getGlyph(c);
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (glyph[col] & (1 << row)) {
                    for (int dy = 0; dy < scale; dy++) {
                        for (int dx = 0; dx < scale; dx++) {
                            int px = x + col * scale + dx;
                            int py = y + row * scale + dy;
                            if (px >= 0 && px < width_ && py >= 0 && py < height_) {
                                img_buf_[py * row_bytes + px / 8] &= ~(0x80 >> (px % 8));
                            }
                        }
                    }
                }
            }
        }
        x += (5 * scale + scale);
    }
}

void Ssd1683Display::drawPixel(int x, int y, bool black) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return;
    }
    int row_bytes = width_ / 8;
    int index = y * row_bytes + x / 8;
    int bit = 7 - (x % 8);
    if (black) {
        img_buf_[index] &= ~(0x80 >> bit);
    } else {
        img_buf_[index] |= (0x80 >> bit);
    }
}

void Ssd1683Display::fillRect(int x, int y, int w, int h) {
    int row_bytes = width_ / 8;
    for (int py = y; py < y + h; py++) {
        if (py < 0 || py >= height_) continue;
        for (int px = x; px < x + w; px++) {
            if (px < 0 || px >= width_) continue;
            img_buf_[py * row_bytes + px / 8] &= ~(0x80 >> (px % 8));
        }
    }
}

int Ssd1683Display::textWidth(int char_count, int scale) {
    return char_count * (5 * scale + scale) - scale;
}

void Ssd1683Display::clearBuffer() {
    memset(img_buf_, 0xFF, spi_data_.buffer_len);
}

void Ssd1683Display::spi_gpio_init() {
    int rst = spi_data_.rst;
    int cs = spi_data_.cs;
    int dc = spi_data_.dc;
    int busy = spi_data_.busy;

    gpio_config_t gpio_conf = {};
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (0x1ULL << rst) | (0x1ULL << dc) | (0x1ULL << cs);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));

    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (0x1ULL << busy);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_conf));

    set_rst_1();
}

void Ssd1683Display::spi_port_init() {
    int mosi = spi_data_.mosi;
    int scl = spi_data_.scl;
    int spi_host = spi_data_.spi_host;
    esp_err_t ret;
    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = -1;
    bus_cfg.mosi_io_num = mosi;
    bus_cfg.sclk_io_num = scl;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = width_ * height_;

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.spics_io_num = -1;
    dev_cfg.clock_speed_hz = 4 * 1000 * 1000;
    dev_cfg.mode = 0;
    dev_cfg.queue_size = 7;

    ret = spi_bus_initialize((spi_host_device_t)spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device((spi_host_device_t)spi_host, &dev_cfg, &spi_);
    ESP_ERROR_CHECK(ret);
}

void Ssd1683Display::read_busy() {
    display_busy_ = true;
    int busy = spi_data_.busy;
    while (gpio_get_level((gpio_num_t)busy) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    display_busy_ = false;
}

void Ssd1683Display::SPI_SendByte(uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    ret = spi_device_polling_transmit(spi_, &t);
    assert(ret == ESP_OK);
}

void Ssd1683Display::EPD_SendData(uint8_t data) {
    set_dc_1();
    set_cs_0();
    SPI_SendByte(data);
    set_cs_1();
}

void Ssd1683Display::EPD_SendCommand(uint8_t command) {
    set_dc_0();
    set_cs_0();
    SPI_SendByte(command);
    set_cs_1();
}

void Ssd1683Display::writeBytes(const uint8_t* buffer, int len) {
    set_dc_1();
    set_cs_0();
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * len;
    t.tx_buffer = buffer;
    ret = spi_device_polling_transmit(spi_, &t);
    assert(ret == ESP_OK);
    set_cs_1();
}

void Ssd1683Display::EPD_SetFullWindow() {
    EPD_SendCommand(0x44);
    EPD_SendData(0);
    EPD_SendData((width_ - 1) / 8);
    EPD_SendCommand(0x45);
    EPD_SendData(0);
    EPD_SendData(0);
    EPD_SendData((height_ - 1) & 0xFF);
    EPD_SendData((height_ - 1) >> 8);
}

void Ssd1683Display::EPD_TurnOnDisplay() {
    EPD_SendCommand(0x22);
    EPD_SendData(0xF7);
    EPD_SendCommand(0x20);
    read_busy();
}

void Ssd1683Display::EPD_TurnOnDisplayFast() {
    EPD_SendCommand(0x22);
    EPD_SendData(0xC7);
    EPD_SendCommand(0x20);
    read_busy();
}

void Ssd1683Display::EPD_Reset() {
    set_rst_1();
    vTaskDelay(pdMS_TO_TICKS(200));
    set_rst_0();
    vTaskDelay(pdMS_TO_TICKS(5));
    set_rst_1();
    vTaskDelay(pdMS_TO_TICKS(200));
}

void Ssd1683Display::EPD_Init() {
    EPD_Reset();
    read_busy();
    
    EPD_SendCommand(0x12);
    read_busy();
    
    EPD_SendCommand(0x21);
    EPD_SendData(0x40);
    EPD_SendData(0x00);
    
    EPD_SendCommand(0x3C);
    EPD_SendData(0x05);
    
    EPD_SendCommand(0x11);
    EPD_SendData(0x03);
    
    EPD_SendCommand(0x44);
    EPD_SendData(0x00);
    EPD_SendData((width_ - 1) / 8);
    
    EPD_SendCommand(0x45);
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    EPD_SendData((height_ - 1) & 0xFF);
    EPD_SendData(((height_ - 1) >> 8) & 0xFF);
    
    EPD_SendCommand(0x4E);
    EPD_SendData(0x00);
    
    EPD_SendCommand(0x4F);
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    
    read_busy();
}

void Ssd1683Display::EPD_InitFast() {
    EPD_Reset();
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
    
    EPD_SendCommand(0x44);
    EPD_SendData(0x00);
    EPD_SendData((width_ - 1) / 8);
    
    EPD_SendCommand(0x45);
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    EPD_SendData((height_ - 1) & 0xFF);
    EPD_SendData(((height_ - 1) >> 8) & 0xFF);
    
    EPD_SendCommand(0x4E);
    EPD_SendData(0x00);
    
    EPD_SendCommand(0x4F);
    EPD_SendData(0x00);
    EPD_SendData(0x00);
    
    read_busy();
}

void Ssd1683Display::EPD_Clear() {
    clearBuffer();
    EPD_SendCommand(0x24);
    writeBytes(img_buf_, spi_data_.buffer_len);
    EPD_SendCommand(0x26);
    writeBytes(img_buf_, spi_data_.buffer_len);
    EPD_TurnOnDisplay();
}

void Ssd1683Display::EPD_Display(const uint8_t* image) {
    EPD_SendCommand(0x24);
    writeBytes(image, spi_data_.buffer_len);
    
    EPD_SendCommand(0x26);
    writeBytes(image, spi_data_.buffer_len);
    
    EPD_TurnOnDisplay();
    refresh_count_ = 0;
}

void Ssd1683Display::EPD_DisplayFast(const uint8_t* image) {
    EPD_SendCommand(0x24);
    writeBytes(image, spi_data_.buffer_len);
    
    EPD_SendCommand(0x26);
    writeBytes(image, spi_data_.buffer_len);
    
    if (refresh_count_ % FULL_REFRESH_INTERVAL == 0) {
        EPD_TurnOnDisplay();
        refresh_count_ = 0;
    } else {
        EPD_TurnOnDisplayFast();
    }
    refresh_count_++;
}

void Ssd1683Display::EPD_Sleep() {
    EPD_SendCommand(0x10);
    EPD_SendData(0x01);
}
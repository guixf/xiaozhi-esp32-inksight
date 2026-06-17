#include "ssd1683_display.h"
#include <esp_log.h>
#include <string.h>
#include <cstdio>
#include <esp_http_client.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <esp_tls.h>
#include <esp_crt_bundle.h>
#include <esp_sntp.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <sys/time.h>
#include <time.h>
#include "boards/common/board.h"
#include "application.h"
#include "device_state.h"
#include "assets/lang_config.h"
#include "wifi_manager.h"

// 使用 SSD1683 芯片默认 LUT（内部 OTP 存储）
// 自定义 LUT 已移除，使用芯片出厂默认值

static const char* TAG = "Ssd1683Display";

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
      refresh_count_(0), display_state_(DisplayState::IDLE),
      voice_state_(VoiceState::INACTIVE),
      led_blink_timer_(nullptr), led_mode_(LED_MODE_OFF),
      led_state_on_(false), double_blink_count_(0),
      voice_icon_x_(-1), voice_icon_y_(0),
      inksight_fetch_pending_(false),
      display_busy_(false), display_pending_(false),
      spi_initialized_(false), pending_image_(nullptr),
      last_voice_activity_ms_(0) {
    img_buf_ = (uint8_t*)heap_caps_malloc(spi_data.buffer_len, MALLOC_CAP_SPIRAM);
    assert(img_buf_);
    
    // 为三色显示分配红色缓冲区
    red_buf_ = (uint8_t*)heap_caps_malloc(spi_data.buffer_len, MALLOC_CAP_SPIRAM);
    assert(red_buf_);
    
    clearBuffer();
    spi_gpio_init();
    spi_port_init();
    EPD_Init();
    
    // ========== 红色显示测试 ==========
    // 强制全屏显示红色，验证硬件是否正常
    ESP_LOGI(TAG, "[TEST] Forcing full red screen to verify hardware");
    memset(red_buf_, 0x00, spi_data_.buffer_len);   // 红色全显（0x00表示显示红色）
    memset(img_buf_, 0xFF, spi_data_.buffer_len);   // 黑白全白（0xFF表示白色）
    RequestDisplayUpdate();
    vTaskDelay(pdMS_TO_TICKS(5000));  // 等待5秒让用户观察
    // ========== 测试结束 ==========
    
    // 初始化 GPIO2 状态指示灯
    InitializeStatusLed();
}

Ssd1683Display::~Ssd1683Display() {
    if (img_buf_) {
        heap_caps_free(img_buf_);
    }
    if (red_buf_) {
        heap_caps_free(red_buf_);
    }
    if (pending_image_) {
        free(pending_image_);
    }
    if (led_blink_timer_) {
        esp_timer_delete(led_blink_timer_);
    }
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
    ret = spi_bus_add_device((spi_host_device_t)spi_host, &dev_cfg, &spi_handle_);
    ESP_ERROR_CHECK(ret);
}

void Ssd1683Display::SPI_SendByte(uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    ret = spi_device_polling_transmit(spi_handle_, &t);
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
    ret = spi_device_polling_transmit(spi_handle_, &t);
    assert(ret == ESP_OK);
    set_cs_1();
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

    EPD_SendCommand(0x01);  // Driver output control (关键：配置显示高度)
    EPD_SendData((height_ - 1) & 0xFF);
    EPD_SendData(((height_ - 1) >> 8) & 0xFF);
    EPD_SendData(0x00);

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

    // 使用 SSD1683 默认 LUT（从内部 OTP 自动加载），不下载自定义 LUT

    read_busy();
    ESP_LOGI(TAG, "EPD initialized (full mode, BWR color)");
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
    ESP_LOGI(TAG, "EPD initialized (fast mode)");
}

void Ssd1683Display::read_busy() {
    display_busy_ = true;
    int busy = spi_data_.busy;
    int timeout_ms = 60000;  // 60秒超时
    int elapsed_ms = 0;
    while (gpio_get_level((gpio_num_t)busy) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
        if (elapsed_ms >= timeout_ms) {
            ESP_LOGE(TAG, "EPD BUSY timeout after %d ms", timeout_ms);
            break;
        }
    }
    display_busy_ = false;
}

void Ssd1683Display::EPD_SetFullWindow() {
    EPD_SendCommand(0x44);
    EPD_SendData(0);
    EPD_SendData((width_ - 1) / 8);
    EPD_SendCommand(0x45);
    EPD_SendData(0);
    EPD_SendData(0);
    EPD_SendData((height_ - 1) & 0xFF);
    EPD_SendData(((height_ - 1) >> 8));
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
    EPD_SendCommand(0x04);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (gpio_get_level((gpio_num_t)spi_data_.busy) == 1) {
        ESP_LOGE(TAG, "EPD_Display: Power on failed, BUSY still HIGH");
        return;
    }
    read_busy();

    EPD_SendCommand(0x24);  // Write Black/White RAM
    writeBytes(image, spi_data_.buffer_len);

    EPD_SendCommand(0x26);  // Write RED RAM
    if (red_buf_) {
        writeBytes(red_buf_, spi_data_.buffer_len);
    } else {
        writeBytes(image, spi_data_.buffer_len);
    }

    EPD_SendCommand(0x22);  // Display Update Control 2
    EPD_SendData(0xF7);     // Full update sequence
    EPD_SendCommand(0x20);  // Activate Display Update Sequence
    read_busy();

    EPD_SendCommand(0x02);  // Power Off
    read_busy();

    refresh_count_ = 0;
    ESP_LOGI(TAG, "Display updated (full, cycle %d)", refresh_count_);
}

void Ssd1683Display::EPD_DisplayFast(const uint8_t* image) {
    EPD_SendCommand(0x24);
    writeBytes(image, spi_data_.buffer_len);

    EPD_SendCommand(0x26);
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

    if (refresh_count_ % 10 == 0) {
        EPD_TurnOnDisplay();
        refresh_count_ = 0;
    } else {
        EPD_TurnOnDisplayFast();
    }
    refresh_count_++;
    ESP_LOGI(TAG, "Display updated 快刷");
}

void Ssd1683Display::EPD_Sleep() {
    EPD_SendCommand(0x10);
    EPD_SendData(0x01);
    vTaskDelay(pdMS_TO_TICKS(200));  // 等待200ms让墨水屏完成休眠
    ESP_LOGI(TAG, "EPD entered sleep mode");
}

void Ssd1683Display::clearBuffer() {
    if (img_buf_) {
        memset(img_buf_, 0xFF, spi_data_.buffer_len);
    }
    if (red_buf_) {
        memset(red_buf_, 0xFF, spi_data_.buffer_len);
    }
}

void Ssd1683Display::RequestDisplayUpdate() {
    if (display_state_ == DisplayState::IDLE) {
        bool use_fast_refresh = always_active_ && !red_buf_;
        if (red_buf_) {
            ESP_LOGI(TAG, "[DISPLAY] Color display detected, forcing full refresh");
        }
        if (use_fast_refresh) {
            ESP_LOGI(TAG, "[DISPLAY] Always active mode, using fast refresh");
            EPD_DisplayFast(img_buf_);
        } else {
            ESP_LOGI(TAG, "[DISPLAY] Using full refresh");
            EPD_Display(img_buf_);
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

void Ssd1683Display::ProcessDisplayUpdate() {
    switch (display_state_) {
        case DisplayState::IDLE:
            if (display_pending_ && pending_image_) {
                bool use_fast_refresh = always_active_ && !red_buf_;
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
    return display_state_ != DisplayState::IDLE;
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

void Ssd1683Display::TriggerImageUpdate() {
    if (external_image_callback_ && img_buf_) {
        external_image_callback_(img_buf_);
    }
}

void Ssd1683Display::SetStatus(const char* status) {
    ESP_LOGW(TAG, "SetStatus: %s", status);
    
    // 检测配网模式状态
    if (status && strcmp(status, Lang::Strings::WIFI_CONFIG_MODE) == 0) {
        ESP_LOGI(TAG, "[WIFI_CONFIG] Entering WiFi config mode display");
        // 从WifiManager获取热点名称和URL
        auto& wifi_manager = WifiManager::GetInstance();
        std::string ap_name = wifi_manager.GetApSsid();
        std::string web_url = wifi_manager.GetApWebUrl();
        
        // 提取IP地址（去掉http://前缀）
        std::string ip_address = web_url;
        if (ip_address.find("http://") == 0) {
            ip_address = ip_address.substr(7);  // 去掉 "http://"
        }
        
        ESP_LOGI(TAG, "[WIFI_CONFIG] AP name: %s, IP: %s", ap_name.c_str(), ip_address.c_str());
        showWifiConfigScreen(ap_name.c_str(), ip_address.c_str());
        
        // 配网模式：双闪
        SetStatusLedMode(LED_MODE_DOUBLE_BLINK);
        return;
    }
    
    // 根据状态设置 LED
    if (status && (
        strcmp(status, "待命") == 0 || 
        strcmp(status, "standby") == 0 || 
        strcmp(status, "STANDBY") == 0 ||
        strcmp(status, "Idle") == 0 ||
        strcmp(status, "idle") == 0 ||
        strcmp(status, "IDLE") == 0
    )) {
        // 待机状态：熄灭 LED
        SetStatusLedMode(LED_MODE_OFF);
    } else if (status && (
        strcmp(status, "正在升级系统...") == 0 ||
        strcmp(status, "UPGRADING") == 0 ||
        strcmp(status, "Upgrading") == 0
    )) {
        // 升级中：呼吸灯
        SetStatusLedMode(LED_MODE_PULSE);
    } else if (status && (
        strcmp(status, "LISTENING") == 0 ||
        strcmp(status, "Listening") == 0 ||
        strcmp(status, "listening") == 0 ||
        strcmp(status, "聆听中") == 0
    )) {
        // 聆听中：常亮
        SetStatusLedMode(LED_MODE_ON);
    } else if (status && (
        strcmp(status, "SPEAKING") == 0 ||
        strcmp(status, "Speaking") == 0 ||
        strcmp(status, "speaking") == 0 ||
        strcmp(status, "说话中") == 0
    )) {
        // 说话中：慢速闪烁（4秒周期）
        SetStatusLedMode(LED_MODE_BLINK_SLOW);
    } else if (status && (
        strcmp(status, "ERROR") == 0 ||
        strcmp(status, "Error") == 0 ||
        strcmp(status, "error") == 0 ||
        strcmp(status, "错误") == 0
    )) {
        // 错误状态：双闪
        SetStatusLedMode(LED_MODE_DOUBLE_BLINK);
    } else if (status && (
        strcmp(status, "NETWORK_DISCONNECTED") == 0 ||
        strcmp(status, "网络断开") == 0 ||
        strcmp(status, "disconnected") == 0
    )) {
        // 网络断开：快速闪烁
        SetStatusLedMode(LED_MODE_BLINK_FAST);
    } else {
        // 默认状态：常亮（连接中、未知状态等）
        SetStatusLedMode(LED_MODE_ON);
    }
    
    // 当进入待命状态时，获取 inksight 图片（除非被抑制）
    if (!suppress_auto_image_fetch_ && status && (
        strcmp(status, "待命") == 0 || 
        strcmp(status, "standby") == 0 || 
        strcmp(status, "STANDBY") == 0 ||
        strcmp(status, "Idle") == 0 ||
        strcmp(status, "idle") == 0 ||
        strcmp(status, "IDLE") == 0
    )) {
        ESP_LOGI(TAG, "[INKSIGHT] Standby state detected, fetching image...");
        // 先同步发送心跳，确保服务器收到状态更新
        DoSendHeartbeat(true);
        // 心跳完成后，获取focus_listening配置
        FetchFocusListening();
        // 然后获取 Inksight 图片（根据always_active值决定刷新模式）
        FetchInksightImage();
    }
}

void Ssd1683Display::SetSuppressAutoImageFetch(bool suppress) {
    ESP_LOGI(TAG, "[INKSIGHT] SetSuppressAutoImageFetch: %d", suppress);
    suppress_auto_image_fetch_ = suppress;
}

void Ssd1683Display::ShowNotification(const char* notification, int duration_ms) {
    ESP_LOGW(TAG, "ShowNotification: %s", notification);
    
    // 根据通知类型设置 LED
    if (notification) {
        if (strstr(notification, "扫描 Wi-Fi") != nullptr ||
            strstr(notification, "SCANNING_WIFI") != nullptr ||
            strstr(notification, "Scanning Wi-Fi") != nullptr ||
            strstr(notification, "连接热点") != nullptr ||
            strstr(notification, "Connect to hotspot") != nullptr ||
            strstr(notification, "切换到") != nullptr ||
            strstr(notification, "Switching to") != nullptr) {
            // WiFi扫描/连接中：快速闪烁（500ms周期）
            SetStatusLedMode(LED_MODE_BLINK_FAST);
        }
        // 其他通知（音量调节等）不改变LED状态
    }
}

// GPIO2 状态指示灯控制实现

void Ssd1683Display::InitializeStatusLed() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << STATUS_LED_GPIO;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    
    // 开机闪烁一下（500ms亮，500ms灭）
    gpio_set_level(STATUS_LED_GPIO, 0);  // 点亮
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(STATUS_LED_GPIO, 1);  // 熄灭
    
    // 初始状态：熄灭（高电平）
    led_mode_ = LED_MODE_OFF;
    led_state_on_ = false;
    double_blink_count_ = 0;
    
    ESP_LOGI(TAG, "[LED] Status LED initialized on GPIO2");
}

void Ssd1683Display::SetStatusLedMode(int mode) {
    // 如果模式相同，不需要重复设置
    if (led_mode_ == mode) {
        return;
    }
    
    // 停止闪烁定时器
    if (led_blink_timer_) {
        esp_timer_stop(led_blink_timer_);
    }
    
    led_mode_ = mode;
    double_blink_count_ = 0;
    
    switch (mode) {
        case LED_MODE_OFF:
            gpio_set_level(STATUS_LED_GPIO, 1);  // 高电平熄灭
            led_state_on_ = false;
            ESP_LOGI(TAG, "[LED] Mode: OFF");
            break;
            
        case LED_MODE_ON:
            gpio_set_level(STATUS_LED_GPIO, 0);  // 低电平点亮
            led_state_on_ = true;
            ESP_LOGI(TAG, "[LED] Mode: ON");
            break;
            
        case LED_MODE_BLINK:
            // 创建闪烁定时器（2秒周期）
            if (!led_blink_timer_) {
                esp_timer_create_args_t args = {};
                args.callback = &LedBlinkTimerCallback;
                args.arg = this;
                args.name = "led_blink_timer";
                esp_timer_create(&args, &led_blink_timer_);
            }
            led_state_on_ = false;
            gpio_set_level(STATUS_LED_GPIO, 0);  // 开始时点亮
            esp_timer_start_periodic(led_blink_timer_, 2000000);  // 2秒周期
            ESP_LOGI(TAG, "[LED] Mode: BLINK (2s on/off)");
            break;
            
        case LED_MODE_BLINK_FAST:
            // 快速闪烁（500ms周期）
            if (!led_blink_timer_) {
                esp_timer_create_args_t args = {};
                args.callback = &LedBlinkTimerCallback;
                args.arg = this;
                args.name = "led_blink_timer";
                esp_timer_create(&args, &led_blink_timer_);
            }
            led_state_on_ = false;
            gpio_set_level(STATUS_LED_GPIO, 0);  // 开始时点亮
            esp_timer_start_periodic(led_blink_timer_, 500000);  // 500ms周期
            ESP_LOGI(TAG, "[LED] Mode: BLINK_FAST (500ms on/off)");
            break;
            
        case LED_MODE_BLINK_SLOW:
            // 慢速闪烁（4秒周期）
            if (!led_blink_timer_) {
                esp_timer_create_args_t args = {};
                args.callback = &LedBlinkTimerCallback;
                args.arg = this;
                args.name = "led_blink_timer";
                esp_timer_create(&args, &led_blink_timer_);
            }
            led_state_on_ = false;
            gpio_set_level(STATUS_LED_GPIO, 0);  // 开始时点亮
            esp_timer_start_periodic(led_blink_timer_, 4000000);  // 4秒周期
            ESP_LOGI(TAG, "[LED] Mode: BLINK_SLOW (4s on/off)");
            break;
            
        case LED_MODE_DOUBLE_BLINK:
            // 双闪模式：快速闪两次，然后暂停
            if (!led_blink_timer_) {
                esp_timer_create_args_t args = {};
                args.callback = &LedBlinkTimerCallback;
                args.arg = this;
                args.name = "led_blink_timer";
                esp_timer_create(&args, &led_blink_timer_);
            }
            double_blink_count_ = 0;
            led_state_on_ = false;
            gpio_set_level(STATUS_LED_GPIO, 0);  // 开始时点亮
            esp_timer_start_periodic(led_blink_timer_, 300000);  // 300ms周期
            ESP_LOGI(TAG, "[LED] Mode: DOUBLE_BLINK");
            break;
            
        case LED_MODE_PULSE:
            // 呼吸灯模式（1秒周期）
            if (!led_blink_timer_) {
                esp_timer_create_args_t args = {};
                args.callback = &LedBlinkTimerCallback;
                args.arg = this;
                args.name = "led_blink_timer";
                esp_timer_create(&args, &led_blink_timer_);
            }
            led_state_on_ = false;
            gpio_set_level(STATUS_LED_GPIO, 0);  // 开始时点亮
            esp_timer_start_periodic(led_blink_timer_, 1000000);  // 1秒周期
            ESP_LOGI(TAG, "[LED] Mode: PULSE (1s on/off)");
            break;
    }
}

void Ssd1683Display::LedBlinkTimerCallback(void* arg) {
    Ssd1683Display* self = static_cast<Ssd1683Display*>(arg);
    
    if (self->led_mode_ == LED_MODE_DOUBLE_BLINK) {
        // 双闪模式：快速闪两次，然后暂停较长时间
        self->double_blink_count_++;
        if (self->double_blink_count_ <= 2) {
            // 前两次：快速闪烁
            self->led_state_on_ = !self->led_state_on_;
            gpio_set_level(STATUS_LED_GPIO, self->led_state_on_ ? 0 : 1);
        } else if (self->double_blink_count_ == 3) {
            // 第三次：熄灭并保持较长时间
            gpio_set_level(STATUS_LED_GPIO, 1);
            self->led_state_on_ = false;
        } else if (self->double_blink_count_ >= 8) {
            // 重置计数，开始新一轮双闪
            self->double_blink_count_ = 0;
            gpio_set_level(STATUS_LED_GPIO, 0);  // 点亮
            self->led_state_on_ = true;
        }
    } else {
        // 普通闪烁模式
        self->led_state_on_ = !self->led_state_on_;
        gpio_set_level(STATUS_LED_GPIO, self->led_state_on_ ? 0 : 1);
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
    if (strcmp(state, "IDLE") == 0) return "XiaoZhiInkSIGHT";
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
    if (state && (strcmp(state, "LISTENING") == 0 || strcmp(state, "SPEAKING") == 0)) {
        ESP_LOGI(TAG, "[CHAT] Device is in listening/speaking state, skipping display update");
        return;
    }
    
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
    last_voice_activity_ms_ = esp_timer_get_time() / 1000; // 更新语音活动时间
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
    last_voice_activity_ms_ = esp_timer_get_time() / 1000; // 更新语音活动时间
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

void Ssd1683Display::showWifiConfigScreen(const char* ap_name, const char* web_url) {
    clearBuffer();

    int marginX = width_ * 7 / 100;
    int contentWidth = width_ - marginX * 2;
    if (contentWidth < 40) contentWidth = width_ - 8;

    // 标题：XIAOZHI INKSIGHT
    const char* title = "XiaoZhiInkSIGHT";
    int titleScale = (height_ < 200) ? 2 : 3;
    int titleX = (width_ - textWidth(strlen(title), titleScale)) / 2;
    if (titleX < marginX) titleX = marginX;
    int titleY = height_ * 12 / 100;
    drawText(title, titleX, titleY, titleScale);

    // 横线
    int lineY = titleY + 7 * titleScale + titleScale * 3;
    drawHLine(marginX, width_ - marginX, lineY);
    drawHLine(marginX, width_ - marginX, lineY + 3);

    // 内容区域
    int bodyScale = (height_ < 200) ? 1 : 2;
    int bodyY = lineY + ((height_ < 200) ? 12 : 20);
    int lineHeight = 7 * bodyScale + bodyScale * 2;

    // 第一行：connect <ap_name>
    char line1[128];
    snprintf(line1, sizeof(line1), "connect %s", ap_name ? ap_name : "Xiaozhi-XXXX");
    int line1X = (width_ - textWidth(strlen(line1), bodyScale)) / 2;
    if (line1X < marginX) line1X = marginX;
    drawText(line1, line1X, bodyY, bodyScale);
    bodyY += lineHeight;

    // 第二行：browser to <ip_address>
    char line2[128];
    snprintf(line2, sizeof(line2), "browser to %s", web_url ? web_url : "192.168.4.1");
    int line2X = (width_ - textWidth(strlen(line2), bodyScale)) / 2;
    if (line2X < marginX) line2X = marginX;
    drawText(line2, line2X, bodyY, bodyScale);

    RequestDisplayUpdate();
    ESP_LOGI(TAG, "[WIFI_CONFIG] Screen shown: %s, %s", ap_name, web_url);
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
        ESP_LOGI(TAG, "[INKSIGHT] Token saved: %s", token);
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
    ESP_LOGI(TAG, "[INKSIGHT] Token cleared");
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
        ESP_LOGI(TAG, "[INKSIGHT] Loaded token from storage");
        return true;
    }

    ESP_LOGI(TAG, "[INKSIGHT] Need to fetch new token from server");
    
    // 检查 WiFi 连接状态
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "[INKSIGHT] WiFi connected to: %s, RSSI: %d", ap_info.ssid, ap_info.rssi);
    } else {
        ESP_LOGE(TAG, "[INKSIGHT] WiFi not connected!");
        return false;
    }
    
    // 获取 IP 地址
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info) == ESP_OK) {
        char ip_str[16];
        inet_ntoa_r(ip_info.ip.addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "[INKSIGHT] IP address: %s", ip_str);
    }
    
    std::string mac = GetMacAddress();
    char url[512];
    snprintf(url, sizeof(url), "%s/api/device/%s/token", inksight_server_.c_str(), mac.c_str());
    ESP_LOGI(TAG, "[INKSIGHT] Fetch token from: %s", url);

    vTaskDelay(pdMS_TO_TICKS(1200));

    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 15000;
    config.crt_bundle_attach = esp_crt_bundle_attach;  // 使用 ESP-IDF 证书包验证
    config.skip_cert_common_name_check = true;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "[INKSIGHT] HTTP client init failed");
        return false;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    bool success = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        esp_err_t err = esp_http_client_open(client, 2);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "[INKSIGHT] Failed to open connection: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(800));
            continue;
        }

        int written = esp_http_client_write(client, "{}", 2);
        if (written < 0) {
            ESP_LOGW(TAG, "[INKSIGHT] Failed to write POST data");
            esp_http_client_close(client);
            vTaskDelay(pdMS_TO_TICKS(800));
            continue;
        }

        int content_len = esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "[INKSIGHT] Token fetch attempt %d, status: %d, len: %d", attempt + 1, status_code, content_len);

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
                
                ESP_LOGI(TAG, "[INKSIGHT] Response (%d bytes): %s", total_read, buffer);
                
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
    ESP_LOGI(TAG, "[INKSIGHT] Config set: server=%s, token=%s", 
             inksight_server_.c_str(), 
             inksight_device_token_.empty() ? "(empty)" : "(present)");
}

void Ssd1683Display::SetBatteryVoltage(int voltage_mv) {
    battery_voltage_mv_ = voltage_mv;
    ESP_LOGD(TAG, "[INKSIGHT] Battery voltage updated: %d mV", voltage_mv);
}

void Ssd1683Display::SetRuntimeModeChangedCallback(std::function<void(bool is_active)> callback) {
    runtime_mode_changed_callback_ = callback;
    ESP_LOGI(TAG, "[INKSIGHT] Runtime mode changed callback set");
}

void Ssd1683Display::SetDeepSleepCallback(std::function<void()> callback) {
    enter_deep_sleep_callback_ = callback;
    ESP_LOGI(TAG, "[INKSIGHT] Deep sleep callback set");
}

int Ssd1683Display::GetSleepMinutes() {
    // 如果配置获取失败，使用2分钟作为回退值
    if (!config_fetched_) {
        ESP_LOGW(TAG, "[RUNTIME] Config not fetched, using fallback sleep interval: 2 minutes");
        return 2;
    }
    return sleep_minutes_;
}

std::string Ssd1683Display::GetRuntimeMode() {
    return runtime_mode_;
}

void Ssd1683Display::SetAlwaysActive(bool active) {
     if (always_active_ != active) {
         always_active_ = active;
         runtime_mode_ = active ? "active" : "interval";  // 同步更新运行模式
         ESP_LOGI(TAG, "[RUNTIME] Always active mode: %s, runtime_mode set to: %s", 
                  active ? "ON" : "OFF", runtime_mode_.c_str());
         
         // 持久化存储到 NVS
         nvs_handle_t nvs_handle;
         esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
         if (err == ESP_OK) {
             nvs_set_u8(nvs_handle, "always_active", active ? 1 : 0);
             nvs_commit(nvs_handle);
             nvs_close(nvs_handle);
             ESP_LOGI(TAG, "[RUNTIME] Saved always_active to NVS: %d", active ? 1 : 0);
         } else {
             ESP_LOGW(TAG, "[RUNTIME] Failed to save always_active to NVS: %s", esp_err_to_name(err));
         }
         
         // 当 always_active = true 时，通知服务器设备处于活跃状态
         if (active) {
             ESP_LOGI(TAG, "[RUNTIME] Posting active mode to server...");
             PostRuntimeMode("active");
         }
         
         // 控制 live mode 定时器：always_active = true 时启动 5 秒轮询
         if (live_mode_timer_callback_) {
             ESP_LOGI(TAG, "[LIVE MODE] %s live mode timer", active ? "Starting" : "Stopping");
             live_mode_timer_callback_(active);
         }
     }
 }

bool Ssd1683Display::IsAlwaysActive() {
    return always_active_;
}

void Ssd1683Display::LoadAlwaysActiveFromNVS() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        uint8_t val = 0;
        err = nvs_get_u8(nvs_handle, "always_active", &val);
        if (err == ESP_OK) {
            always_active_ = (val != 0);
            ESP_LOGI(TAG, "[RUNTIME] Loaded always_active from NVS: %s", always_active_ ? "ON" : "OFF");
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "[RUNTIME] No always_active in NVS, using default: OFF");
            always_active_ = false;
        } else {
            ESP_LOGW(TAG, "[RUNTIME] Failed to read always_active from NVS: %s", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGW(TAG, "[RUNTIME] Failed to open NVS: %s", esp_err_to_name(err));
    }
}

void Ssd1683Display::SetLiveModeTimerCallback(std::function<void(bool enable)> callback) {
    live_mode_timer_callback_ = callback;
    ESP_LOGI(TAG, "[LIVE MODE] Live mode timer callback set");
}

void Ssd1683Display::PostRuntimeMode(const char* mode) {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[RUNTIME] Server not configured, skip posting runtime mode");
        return;
    }
    
    if (!EnsureDeviceToken()) {
        ESP_LOGE(TAG, "[RUNTIME] Failed to ensure device token");
        return;
    }
    
    // 如果传入了 mode 参数，更新 runtime_mode_
    if (mode != nullptr) {
        runtime_mode_ = mode;
        ESP_LOGI(TAG, "[RUNTIME] Runtime mode set to: %s", mode);
    }
    
    std::string mac = GetMacAddress();
    char url[512];
    snprintf(url, sizeof(url), "%s/api/device/%s/runtime", inksight_server_.c_str(), mac.c_str());
    
    ESP_LOGI(TAG, "[RUNTIME] Posting runtime mode: %s", runtime_mode_.c_str());
    
    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 10000;
    config.crt_bundle_attach = esp_crt_bundle_attach;  // 使用 ESP-IDF 证书包验证
    config.skip_cert_common_name_check = true;
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "[RUNTIME] HTTP client init failed");
        return;
    }
    
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    if (!inksight_device_token_.empty()) {
        esp_http_client_set_header(client, "X-Device-Token", inksight_device_token_.c_str());
    }
    
    char body[128];
    snprintf(body, sizeof(body), "{\"mode\":\"%s\"}", runtime_mode_.c_str());
    
    esp_err_t err = esp_http_client_open(client, strlen(body));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[RUNTIME] Failed to open connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return;
    }
    
    int written = esp_http_client_write(client, body, strlen(body));
    if (written < 0) {
        ESP_LOGW(TAG, "[RUNTIME] Failed to write POST data");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return;
    }
    
    esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    
    ESP_LOGI(TAG, "[RUNTIME] POST -> %d", status_code);
    
    esp_http_client_cleanup(client);
}

void Ssd1683Display::FetchInksightImage() {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[INKSIGHT] Server not configured, skip fetch");
        return;
    }
    if (inksight_fetch_pending_) {
        ESP_LOGI(TAG, "[INKSIGHT] Fetch already in progress");
        return;
    }
    inksight_fetch_pending_ = true;
    BaseType_t result = xTaskCreatePinnedToCore(&Ssd1683Display::InksightFetchTask, "inksight_fetch", 12288, this, 5, &inksight_task_handle_, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "[INKSIGHT] Failed to create fetch task");
        inksight_fetch_pending_ = false;
        return;
    }
    ESP_LOGI(TAG, "[INKSIGHT] Fetch task created");
}

void Ssd1683Display::InksightFetchTask(void* param) {
    Ssd1683Display* self = static_cast<Ssd1683Display*>(param);
    ESP_LOGI(TAG, "[INKSIGHT] Fetch task started");
    self->DoFetchInksightImage();
    ESP_LOGI(TAG, "[INKSIGHT] Fetch task completed");
    
    // 显示完图片后，向服务器报告当前的 runtime mode
    ESP_LOGI(TAG, "[RUNTIME] Posting runtime mode: %s", self->runtime_mode_.c_str());
    self->PostRuntimeMode(self->runtime_mode_.c_str());
    
    // 检查是否应该进入深度睡眠
    ESP_LOGI(TAG, "[RUNTIME] Current runtime mode: %s, always_active: %s", 
             self->runtime_mode_.c_str(), self->always_active_ ? "ON" : "OFF");
    
    if (self->runtime_mode_ == "interval") {
        bool wifi_ok = self->IsWifiConnected();
        bool voice_inactive = self->IsVoiceInactive();
        ESP_LOGI(TAG, "[RUNTIME] WiFi: %s, Voice inactive: %s", 
                 wifi_ok ? "OK" : "DISCONNECTED", 
                 voice_inactive ? "YES (>5s)" : "NO");
        
        if (wifi_ok && voice_inactive) {
            ESP_LOGI(TAG, "[RUNTIME] Conditions met (WiFi OK + voice inactive >5s), entering deep sleep now...");
            if (self->enter_deep_sleep_callback_) {
                ESP_LOGI(TAG, "[RUNTIME] Calling deep sleep callback");
                self->enter_deep_sleep_callback_();
            } else {
                ESP_LOGW(TAG, "[RUNTIME] Deep sleep callback not set");
            }
        } else {
            ESP_LOGI(TAG, "[RUNTIME] Skipping deep sleep: WiFi=%s, voice_inactive=%s", 
                     wifi_ok ? "OK" : "NO", voice_inactive ? "YES" : "NO");
        }
    } else {
        ESP_LOGI(TAG, "[RUNTIME] Active mode, will not enter deep sleep");
    }
    
    self->inksight_fetch_pending_ = false;
    vTaskDelete(NULL);
}

void Ssd1683Display::DoFetchInksightImage() {
    if (inksight_server_.empty() || !img_buf_) {
        ESP_LOGW(TAG, "[INKSIGHT] Skip fetch: server empty or buffer null");
        return;
    }

    if (!EnsureDeviceToken()) {
        ESP_LOGE(TAG, "[INKSIGHT] Failed to ensure device token, cannot fetch image");
        return;
    }

    std::string mac = GetMacAddress();
    int rssi = 0;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        rssi = ap_info.rssi;
    }

    int epd_bpp = 2;  // SSD1683 BWR 三色屏使用 2bpp
    int color_capability = 3;  // 三色屏(BWR)支持黑、白、红
    char url[512];
    snprintf(url, sizeof(url), "%s/api/render?mac=%s&rssi=%d&refresh_min=%d&w=%d&h=%d&bpp=%d&colors=%d&v=%.2f",
              inksight_server_.c_str(),
              mac.c_str(),
              rssi,
              (int)(HEARTBEAT_INTERVAL_MS / 60000),
              width_,
              height_,
              epd_bpp,
              color_capability,
              (float)battery_voltage_mv_ / 1000.0f);

    for (int attempt = 0; attempt < 3; attempt++) {
        ESP_LOGI(TAG, "[INKSIGHT] Fetching: %s (attempt %d)", url, attempt + 1);

        esp_http_client_config_t config = {};
        config.url = url;
        config.timeout_ms = 30000;
        config.crt_bundle_attach = esp_crt_bundle_attach;  // 使用 ESP-IDF 证书包验证
        config.skip_cert_common_name_check = true;

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (!client) {
            ESP_LOGE(TAG, "[INKSIGHT] HTTP client init failed");
            return;
        }

        if (!inksight_device_token_.empty()) {
            esp_http_client_set_header(client, "X-Device-Token", inksight_device_token_.c_str());
        }
        esp_http_client_set_header(client, "Accept-Encoding", "identity");

        esp_err_t err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "[INKSIGHT] Failed to open connection: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int content_len = esp_http_client_fetch_headers(client);
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "[INKSIGHT] HTTP status: %d, content_len: %d", status_code, content_len);

        if (status_code < 200 || status_code >= 300) {
            ESP_LOGW(TAG, "[INKSIGHT] HTTP request failed: %d", status_code);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            
            if (status_code == 401 && attempt == 0) {
                ESP_LOGW(TAG, "[INKSIGHT] 401 Unauthorized, clearing token and retrying");
                ClearDeviceToken();
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int color_buf_len = (width_ * height_) / 4;  // 2bpp: 4 pixels per byte
        if (content_len == color_buf_len) {
            ESP_LOGI(TAG, "[INKSIGHT] Raw 2bpp format detected, reading directly");
            
            uint8_t* color_buf = (uint8_t*)malloc(color_buf_len);
            if (!color_buf) {
                ESP_LOGE(TAG, "[INKSIGHT] Failed to alloc color_buf");
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            
            if (!readExact(client, color_buf, color_buf_len, 10000)) {
                ESP_LOGW(TAG, "[INKSIGHT] Failed to read 2bpp data");
                free(color_buf);
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            
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
                    

                    
                    if (color == 0x00) {
                        img_buf_[bufIndex] &= ~(0x80 >> bitPos);
                        red_buf_[bufIndex] |= (0x80 >> bitPos);
                    } else if (color == 0x01) {
                        red_buf_[bufIndex] |= (0x80 >> bitPos);
                        img_buf_[bufIndex] |= (0x80 >> bitPos);
                    } else if (color == 0x02) {
                        img_buf_[bufIndex] |= (0x80 >> bitPos);
                        red_buf_[bufIndex] |= (0x80 >> bitPos);
                    } else if (color == 0x03) {
                        img_buf_[bufIndex] |= (0x80 >> bitPos);
                        red_buf_[bufIndex] &= ~(0x80 >> bitPos);
                    }
                }
            }
            
            
            
            // 在图片数据处理完成后（例如2bpp转换循环之后），添加如下代码：
int red_pixel_count = 0;
for (int i = 0; i < buf_len; i++) {
    if (red_buf_[i] != 0xFF) {
        // 只要该字节里有0，就代表有红色或黑色像素
        uint8_t non_white = ~red_buf_[i];
        red_pixel_count += __builtin_popcount(non_white);
    }
}
ESP_LOGI(TAG, "[SELF-CHECK] Red pixel count in buffer: %d", red_pixel_count);


            ESP_LOGI(TAG, "[INKSIGHT] 2BPP conversion complete, red_buf_ initialized");
            
            int red_pixels = 0;
            int red_bytes = 0;
            int first_red_row = -1;
            int first_red_col = -1;
            int last_red_row = -1;
            int last_red_col = -1;
            
            for (int i = 0; i < buf_len; i++) {
                if (red_buf_[i] != 0xFF) {
                    red_bytes++;
                    uint8_t mask = ~red_buf_[i];
                    int bit_count = __builtin_popcount(mask);
                    red_pixels += bit_count;
                    
                    int byte_row = i / w;
                    int byte_col = i % w;
                    for (int bit = 0; bit < 8; bit++) {
                        if (mask & (0x80 >> bit)) {
                            int pixel_row = byte_row;
                            int pixel_col = byte_col * 8 + bit;
                            
                            if (first_red_row == -1) {
                                first_red_row = pixel_row;
                                first_red_col = pixel_col;
                            }
                            last_red_row = pixel_row;
                            last_red_col = pixel_col;
                        }
                    }
                }
            }
            
            ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] Total red pixels received: %d", red_pixels);
            ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] Bytes with red pixels: %d/%d", red_bytes, buf_len);
            
            if (red_pixels > 0) {
                ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] Red area: row [%d-%d], col [%d-%d]", 
                         first_red_row, last_red_row, first_red_col, last_red_col);
                
                ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] First 10 red bytes (offset, value):");
                int count = 0;
                for (int i = 0; i < buf_len && count < 10; i++) {
                    if (red_buf_[i] != 0xFF) {
                        ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] Offset 0x%04X: 0x%02X (non-red bits: %d)", 
                                 i, red_buf_[i], __builtin_popcount(~red_buf_[i]));
                        count++;
                    }
                }
                
                ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] Checking color_buf raw data (first 20 bytes):");
                for (int i = 0; i < 20 && i < color_buf_len; i++) {
                    ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] color_buf[%d] = 0x%02X", i, color_buf[i]);
                }
            } else {
                ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] WARNING: No red pixels found in received image!");
                ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] Checking color_buf raw data (first 32 bytes):");
                for (int i = 0; i < 32 && i < color_buf_len; i++) {
                    ESP_LOGI(TAG, "[INKSIGHT RED DEBUG] color_buf[%d] = 0x%02X", i, color_buf[i]);
                }
            }
            
            free(color_buf);
            
            auto& app = Application::GetInstance();
            auto state = app.GetDeviceState();
            if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
                ESP_LOGI(TAG, "[INKSIGHT] Device is in listening/speaking state, skipping display update");
            } else {
                RequestDisplayUpdate();
            }
            break;
        }
        ESP_LOGI(TAG, "[INKSIGHT] Not raw 2bpp (%d bytes), fallback to BMP", content_len);

        int max_buf_size = 128 * 1024;
        uint8_t* response_buf = (uint8_t*)malloc(max_buf_size);
        if (!response_buf) {
            ESP_LOGE(TAG, "[INKSIGHT] Failed to alloc response buffer");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        int bytes_read = 0;
        int total_read = 0;
        uint64_t t0 = esp_timer_get_time();
        int consecutive_zero_count = 0;
        while (total_read < max_buf_size) {
            bytes_read = esp_http_client_read(client, (char*)response_buf + total_read, max_buf_size - total_read);
            if (bytes_read > 0) {
                total_read += bytes_read;
                t0 = esp_timer_get_time();
                consecutive_zero_count = 0;
            } else if (bytes_read == 0) {
                consecutive_zero_count++;
                // 如果已经读取了数据且连续多次返回0，说明数据读取完成
                if (total_read > 0 && consecutive_zero_count >= 100) {
                    ESP_LOGI(TAG, "[INKSIGHT] Response read complete, total: %d bytes", total_read);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            } else {
                ESP_LOGW(TAG, "[INKSIGHT] Error reading response: %d", bytes_read);
                break;
            }
            if (esp_timer_get_time() - t0 > 10000000) {
                ESP_LOGW(TAG, "[INKSIGHT] Response read timeout");
                break;
            }
        }
        esp_http_client_close(client);
        esp_http_client_cleanup(client);

        ESP_LOGI(TAG, "[INKSIGHT] Total bytes read: %d", total_read);

        if (total_read < 54) {
            ESP_LOGW(TAG, "[INKSIGHT] Response too short: %d bytes", total_read);
            free(response_buf);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "[INKSIGHT] BMP header bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 response_buf[0], response_buf[1], response_buf[2], response_buf[3],
                 response_buf[4], response_buf[5], response_buf[6], response_buf[7],
                 response_buf[8], response_buf[9], response_buf[10], response_buf[11],
                 response_buf[12], response_buf[13]);

        if (response_buf[0] != 'B' || response_buf[1] != 'M') {
            ESP_LOGW(TAG, "[INKSIGHT] Invalid BMP signature: %c%c", response_buf[0], response_buf[1]);
            free(response_buf);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        memset(img_buf_, 0xFF, spi_data_.buffer_len);
        memset(red_buf_, 0xFF, spi_data_.buffer_len);

        int rowBytes = width_ / 8;
        int rowStride = (rowBytes + 3) & ~3;

        uint32_t pixelOffset = response_buf[10]
                            | ((uint32_t)response_buf[11] << 8)
                            | ((uint32_t)response_buf[12] << 16)
                            | ((uint32_t)response_buf[13] << 24);
        ESP_LOGI(TAG, "[INKSIGHT] BMP pixel offset: %u", pixelOffset);

        int bmpBits = response_buf[28] | ((int)response_buf[29] << 8);
        ESP_LOGI(TAG, "[INKSIGHT] BMP bit count: %d", bmpBits);

        int dataOffset = pixelOffset;

        ESP_LOGI(TAG, "[INKSIGHT] Reading main (BW) image...");
        if (bmpBits <= 1) {
            for (int bmpY = 0; bmpY < height_; bmpY++) {
                if (dataOffset + rowStride > total_read) {
                    ESP_LOGW(TAG, "[INKSIGHT] Not enough data for row %d", bmpY);
                    free(response_buf);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                int dispY = height_ - 1 - bmpY;
                memcpy(img_buf_ + dispY * rowBytes, response_buf + dataOffset, rowBytes);
                dataOffset += rowStride;
            }
        } else {
            int srcRowBytes = (width_ * bmpBits + 31) / 32 * 4;
            for (int bmpY = 0; bmpY < height_; bmpY++) {
                if (dataOffset + srcRowBytes > total_read) {
                    ESP_LOGW(TAG, "[INKSIGHT] Not enough data for row %d", bmpY);
                    free(response_buf);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                int dispY = height_ - 1 - bmpY;
                uint8_t* srcRow = response_buf + dataOffset;
                for (int x = 0; x < width_; x++) {
                    uint8_t pixel;
                    if (bmpBits == 8) {
                        pixel = srcRow[x];
                    } else {
                        pixel = srcRow[x * 3];
                    }
                    if (pixel < 128) {
                        img_buf_[dispY * rowBytes + x / 8] &= ~(0x80 >> (x % 8));
                    }
                }
                dataOffset += srcRowBytes;
            }
        }

        ESP_LOGI(TAG, "[INKSIGHT] Reading red mask image...");
        if (dataOffset + 14 > total_read) {
            ESP_LOGW(TAG, "[INKSIGHT] No red mask data available");
            free(response_buf);
            RequestDisplayUpdate();
            break;
        }

        if (response_buf[dataOffset] != 'B' || response_buf[dataOffset + 1] != 'M') {
            ESP_LOGW(TAG, "[INKSIGHT] No red mask BMP found, assuming 2-color image");
            free(response_buf);
            RequestDisplayUpdate();
            break;
        }

        uint32_t redPixelOffset = response_buf[dataOffset + 10]
                               | ((uint32_t)response_buf[dataOffset + 11] << 8)
                               | ((uint32_t)response_buf[dataOffset + 12] << 16)
                               | ((uint32_t)response_buf[dataOffset + 13] << 24);

        int redDataOffset = dataOffset + redPixelOffset;
        int redBmpBits = response_buf[dataOffset + 28] | ((int)response_buf[dataOffset + 29] << 8);

        ESP_LOGI(TAG, "[INKSIGHT] Reading red mask data...");
        if (redBmpBits <= 1) {
            for (int bmpY = 0; bmpY < height_; bmpY++) {
                if (redDataOffset + rowStride > total_read) {
                    ESP_LOGW(TAG, "[INKSIGHT] Not enough red data for row %d", bmpY);
                    break;
                }
                int dispY = height_ - 1 - bmpY;
                memcpy(red_buf_ + dispY * rowBytes, response_buf + redDataOffset, rowBytes);
                redDataOffset += rowStride;
            }
        } else {
            int srcRowBytes = (width_ * redBmpBits + 31) / 32 * 4;
            for (int bmpY = 0; bmpY < height_; bmpY++) {
                if (redDataOffset + srcRowBytes > total_read) {
                    ESP_LOGW(TAG, "[INKSIGHT] Not enough red data for row %d", bmpY);
                    break;
                }
                int dispY = height_ - 1 - bmpY;
                uint8_t* srcRow = response_buf + redDataOffset;
                for (int x = 0; x < width_; x++) {
                    uint8_t pixel;
                    if (redBmpBits == 8) {
                        pixel = srcRow[x];
                    } else {
                        pixel = srcRow[x * 3];
                    }
                    if (pixel < 128) {
                        red_buf_[dispY * rowBytes + x / 8] &= ~(0x80 >> (x % 8));
                    }
                }
                redDataOffset += srcRowBytes;
            }
        }

        free(response_buf);

        ESP_LOGI(TAG, "[INKSIGHT] Image fetched (3-color mode), triggering display");
        
        auto& app = Application::GetInstance();
        auto state = app.GetDeviceState();
        if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
            ESP_LOGI(TAG, "[INKSIGHT] Device is in listening/speaking state, skipping display update");
        } else {
            RequestDisplayUpdate();
        }
        break;
    }
}

// ── Helper Functions ──────────────────────────────────────────────────

bool Ssd1683Display::readExact(esp_http_client_handle_t client, uint8_t* buf, int len, int timeout_ms) {
    int got = 0;
    uint64_t t0 = esp_timer_get_time();
    while (got < len) {
        int n = esp_http_client_read(client, (char*)buf + got, len - got);
        if (n > 0) {
            got += n;
            t0 = esp_timer_get_time();
        } else if (n == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            ESP_LOGW(TAG, "[INKSIGHT] readExact error: %d", n);
            return false;
        }
        if (esp_timer_get_time() - t0 > (uint64_t)timeout_ms * 1000) {
            ESP_LOGW(TAG, "[INKSIGHT] readExact timeout: %d/%d", got, len);
            return false;
        }
    }
    return true;
}

// ── Heartbeat Functions ────────────────────────────────────────────────

bool Ssd1683Display::IsWifiConnected() {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err != ESP_OK) return false;
    return ap_info.rssi != 0;
}

bool Ssd1683Display::IsVoiceInactive() {
    uint64_t current_time_ms = esp_timer_get_time() / 1000;
    uint64_t inactivity_duration = current_time_ms - last_voice_activity_ms_;
    return inactivity_duration >= VOICE_INACTIVITY_THRESHOLD_MS;
}

struct HeartbeatTaskParams {
    Ssd1683Display* self;
    bool force;
};

void Ssd1683Display::SendHeartbeat(bool force) {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[HEARTBEAT] Server not configured, skip");
        return;
    }

    ESP_LOGI(TAG, "[HEARTBEAT] Sending heartbeat (force=%d)", force);
    HeartbeatTaskParams* params = new HeartbeatTaskParams{this, force};
    BaseType_t result = xTaskCreatePinnedToCore(&Ssd1683Display::HeartbeatTask, "heartbeat", 8192, params, 5, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "[HEARTBEAT] Failed to create heartbeat task");
        delete params;
    }
}

void Ssd1683Display::HeartbeatTask(void* param) {
    HeartbeatTaskParams* params = static_cast<HeartbeatTaskParams*>(param);
    params->self->DoSendHeartbeat(params->force);
    delete params;
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
    bool should_send_heartbeat = force || last_heartbeat_ms_ == 0 || (now - last_heartbeat_ms_) >= HEARTBEAT_INTERVAL_MS;

    if (should_send_heartbeat) {
        if (!EnsureDeviceToken()) {
            ESP_LOGE(TAG, "[HEARTBEAT] Failed to ensure device token");
            // Even if we can't send heartbeat, still check pending refresh
            CheckPendingRefresh();
            return;
        }

        std::string mac = GetMacAddress();
        char url[512];
        snprintf(url, sizeof(url), "%s/api/device/%s/heartbeat", inksight_server_.c_str(), mac.c_str());

        esp_http_client_config_t config = {};
        config.url = url;
        config.timeout_ms = 10000;
        config.crt_bundle_attach = esp_crt_bundle_attach;  // 使用 ESP-IDF 证书包验证
        config.skip_cert_common_name_check = true;

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (!client) {
            ESP_LOGE(TAG, "[HEARTBEAT] HTTP client init failed");
            CheckPendingRefresh();
            return;
        }

        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        if (!inksight_device_token_.empty()) {
            esp_http_client_set_header(client, "X-Device-Token", inksight_device_token_.c_str());
        }

        // 获取电池电压和WiFi RSSI
        float battery_voltage = (float)battery_voltage_mv_ / 1000.0f;
        int wifi_rssi = -60;
        
        // 获取WiFi RSSI
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            wifi_rssi = ap_info.rssi;
        }
        
        char body[200];
        snprintf(body, sizeof(body), "{\"battery_voltage\":%.2f,\"wifi_rssi\":%d,\"always_active\":%s,\"runtime_mode\":\"%s\"}", 
                 battery_voltage, wifi_rssi, 
                 always_active_ ? "true" : "false", 
                 runtime_mode_.c_str());
        ESP_LOGI(TAG, "[HEARTBEAT] Request body: %s", body);

        bool success = false;
        for (int attempt = 0; attempt < 2; attempt++) {
            esp_err_t err = esp_http_client_open(client, strlen(body));
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "[HEARTBEAT] Failed to open connection: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            int written = esp_http_client_write(client, body, strlen(body));
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
                
                // Read response body and check runtime_mode
                char* response_body = nullptr;
                int content_len = esp_http_client_fetch_headers(client);
                ESP_LOGI(TAG, "[RUNTIME] Response content length: %d", content_len);
                
                // 处理响应内容，无论 content_len 是正数还是负数
                // 先读取响应内容（如果可以的话）
                if (content_len > 0 && content_len < 4096) {
                    response_body = (char*)malloc(content_len + 1);
                    if (response_body) {
                        int read_len = esp_http_client_read(client, response_body, content_len);
                        if (read_len > 0) {
                            response_body[read_len] = '\0';
                            ESP_LOGI(TAG, "[RUNTIME] Response: %s", response_body);
                            
                            // Check for always_active in response (local override) - MUST be checked FIRST
                            if (strstr(response_body, "\"always_active\":true") != nullptr ||
                                strstr(response_body, "\"always_active\": true") != nullptr ||
                                strstr(response_body, "\"always_active\":1") != nullptr ||
                                strstr(response_body, "\"always_active\": 1") != nullptr) {
                                ESP_LOGI(TAG, "[RUNTIME] Server enabled always active mode");
                                SetAlwaysActive(true);
                            } else if (strstr(response_body, "\"always_active\":false") != nullptr ||
                                       strstr(response_body, "\"always_active\": false") != nullptr ||
                                       strstr(response_body, "\"always_active\":0") != nullptr ||
                                       strstr(response_body, "\"always_active\": 0") != nullptr) {
                                ESP_LOGI(TAG, "[RUNTIME] Server disabled always active mode");
                                SetAlwaysActive(false);
                            }
                            
                            // 标记配置获取成功
                            config_fetched_ = true;
                            
                            // Check for runtime_mode in response
                            // Only update runtime_mode if always_active is false
                            if (!always_active_) {
                                if (strstr(response_body, "\"runtime_mode\":\"interval\"") != nullptr ||
                                    strstr(response_body, "\"runtime_mode\": \"interval\"") != nullptr) {
                                    ESP_LOGI(TAG, "[RUNTIME] Server requested interval mode");
                                    if (runtime_mode_ != "interval") {
                                        runtime_mode_ = "interval";
                                        ESP_LOGI(TAG, "[RUNTIME] Switched to interval mode");
                                        if (runtime_mode_changed_callback_) {
                                            runtime_mode_changed_callback_(false);
                                        }
                                    }
                                } else if (strstr(response_body, "\"runtime_mode\":\"active\"") != nullptr ||
                                           strstr(response_body, "\"runtime_mode\": \"active\"") != nullptr) {
                                    ESP_LOGI(TAG, "[RUNTIME] Server requested active mode");
                                    if (runtime_mode_ != "active") {
                                        runtime_mode_ = "active";
                                        ESP_LOGI(TAG, "[RUNTIME] Switched to active mode");
                                        if (runtime_mode_changed_callback_) {
                                            runtime_mode_changed_callback_(true);
                                        }
                                    }
                                }
                            } else {
                                ESP_LOGI(TAG, "[RUNTIME] Always active mode is ON, ignoring server runtime_mode request");
                            }
                            
                            // Check for refreshInterval in response (camelCase format)
                            const char* refresh_min_ptr = strstr(response_body, "\"refreshInterval\":");
                            if (!refresh_min_ptr) {
                                // Try snake_case format
                                refresh_min_ptr = strstr(response_body, "\"refresh_interval\":");
                            }
                            if (!refresh_min_ptr) {
                                // Try refresh_min format
                                refresh_min_ptr = strstr(response_body, "\"refresh_min\":");
                            }
                            
                            if (refresh_min_ptr) {
                                // Skip the property name
                                const char* end_name = strchr(refresh_min_ptr, ':');
                                if (end_name) {
                                    refresh_min_ptr = end_name + 1;
                                    while (*refresh_min_ptr == ' ' || *refresh_min_ptr == '"') refresh_min_ptr++;
                                    int new_sleep_min = atoi(refresh_min_ptr);
                                    if (new_sleep_min > 0 && new_sleep_min != sleep_minutes_) {
                                        sleep_minutes_ = new_sleep_min;
                                        ESP_LOGI(TAG, "[RUNTIME] Updated sleep interval to %d minutes (from refreshInterval)", sleep_minutes_);
                                    }
                                }
                            }
                            
                            // Check for focus_listening in response (server config)
                            // When focus_listening is true, device stays active in interval mode
                            bool focus_listening_enabled = false;
                            if (strstr(response_body, "\"is_focus_listening\":true") != nullptr ||
                                strstr(response_body, "\"is_focus_listening\": true") != nullptr ||
                                strstr(response_body, "\"focus_listening\":1") != nullptr ||
                                strstr(response_body, "\"focus_listening\": 1") != nullptr) {
                                focus_listening_enabled = true;
                            }
                            
                            ESP_LOGI(TAG, "[FOCUS] Server focus_listening: %s, Local always_active: %s", 
                                     focus_listening_enabled ? "true" : "false",
                                     always_active_ ? "ON" : "OFF");
                            
                            if (focus_listening_enabled) {
                                ESP_LOGI(TAG, "[FOCUS] Setting always_active to ON (from server)");
                                SetAlwaysActive(true);
                            }
                        }
                        free(response_body);
                    }
                } else {
                    // content_len 为负数或太大，添加日志显示本地状态
                    ESP_LOGI(TAG, "[FOCUS] Server focus_listening: (unknown), Local always_active: %s", 
                             always_active_ ? "ON" : "OFF");
                    ESP_LOGW(TAG, "[RUNTIME] Response content length invalid (%d), cannot parse server response", content_len);
                }
                
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
    } else {
        ESP_LOGD(TAG, "[HEARTBEAT] Skip, interval not reached");
    }

    // Always check pending refresh, regardless of whether we sent a heartbeat
    CheckPendingRefresh();
}

void Ssd1683Display::CheckPendingRefresh() {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[INKSIGHT] Server not configured, skip pending refresh check");
        return;
    }
    if (inksight_fetch_pending_) {
        ESP_LOGI(TAG, "[INKSIGHT] Fetch already in progress, skip pending refresh check");
        return;
    }
    BaseType_t result = xTaskCreatePinnedToCore(&Ssd1683Display::PendingRefreshTask, "pending_refresh", 8192, this, 5, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "[INKSIGHT] Failed to create pending refresh task");
    }
}

void Ssd1683Display::PendingRefreshTask(void* param) {
    Ssd1683Display* self = static_cast<Ssd1683Display*>(param);
    self->DoCheckPendingRefresh();
    vTaskDelete(NULL);
}

void Ssd1683Display::DoCheckPendingRefresh() {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[INKSIGHT] Skip check pending refresh: server empty");
        return;
    }

    if (!EnsureDeviceToken()) {
        ESP_LOGE(TAG, "[INKSIGHT] Failed to ensure device token");
        return;
    }

    std::string mac = GetMacAddress();
    char url[512];
    snprintf(url, sizeof(url), "%s/api/device/%s/state", inksight_server_.c_str(), mac.c_str());

    ESP_LOGI(TAG, "[INKSIGHT] Checking pending refresh: %s", url);

    char* buffer = nullptr;
    int buffer_len = 0;

    if (HttpGetState(url, &buffer, &buffer_len)) {
        bool pendingRefresh = false;

        if (buffer && buffer_len > 0) {
            pendingRefresh =
                strstr(buffer, "\"pending_refresh\":1") != nullptr ||
                strstr(buffer, "\"pending_refresh\": 1") != nullptr ||
                strstr(buffer, "\"pending_refresh\":true") != nullptr ||
                strstr(buffer, "\"pending_refresh\": true") != nullptr;
        }

        if (pendingRefresh) {
            ESP_LOGI(TAG, "[INKSIGHT] Pending refresh detected, triggering fetch");
            FetchInksightImage();
        } else {
            ESP_LOGI(TAG, "[INKSIGHT] No pending refresh");
        }

        free(buffer);
    } else {
        ESP_LOGW(TAG, "[INKSIGHT] Failed to check pending refresh");
    }
}

bool Ssd1683Display::HttpGetState(const char* url, char** out_buffer, int* out_len) {
    if (!url || !out_buffer || !out_len) {
        return false;
    }

    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 10000;
    config.crt_bundle_attach = esp_crt_bundle_attach;  // 使用 ESP-IDF 证书包验证
    config.skip_cert_common_name_check = true;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "[INKSIGHT] HTTP client init failed");
        return false;
    }

    if (!inksight_device_token_.empty()) {
        esp_http_client_set_header(client, "X-Device-Token", inksight_device_token_.c_str());
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[INKSIGHT] Failed to open connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }

    int content_len = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "[INKSIGHT] HTTP state check: %d, len: %d", status_code, content_len);

    if (status_code < 200 || status_code >= 300) {
        ESP_LOGW(TAG, "[INKSIGHT] HTTP request failed: %d", status_code);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    if (content_len > 0 && content_len < 4096) {
        *out_buffer = (char*)malloc(content_len + 1);
        if (*out_buffer) {
            int n = esp_http_client_read(client, *out_buffer, content_len);
            if (n > 0) {
                (*out_buffer)[n] = '\0';
                *out_len = n;
            } else {
                free(*out_buffer);
                *out_buffer = nullptr;
                *out_len = 0;
            }
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    return *out_buffer != nullptr;
}

void Ssd1683Display::FetchFocusListening() {
    if (inksight_server_.empty()) {
        ESP_LOGW(TAG, "[FOCUS] Server not configured, skip");
        return;
    }
    
    if (!IsWifiConnected()) {
        ESP_LOGW(TAG, "[FOCUS] WiFi not connected, skip");
        return;
    }
    
    if (!EnsureDeviceToken()) {
        ESP_LOGE(TAG, "[FOCUS] Failed to ensure device token");
        return;
    }
    
    std::string mac = GetMacAddress();
    char url[512];
    snprintf(url, sizeof(url), "%s/api/config/%s", inksight_server_.c_str(), mac.c_str());
    
    ESP_LOGI(TAG, "[FOCUS] Fetching focus_listening config: %s", url);
    
    char* buffer = nullptr;
    int buffer_len = 0;
    
    if (HttpGetState(url, &buffer, &buffer_len)) {
        bool server_always_active = false;
        bool focus_listening_enabled = false;
        
        if (buffer && buffer_len > 0) {
            ESP_LOGI(TAG, "[FOCUS] Config response: %s", buffer);
            
            // 标记配置获取成功
            config_fetched_ = true;
            
            // Check for refreshInterval in response
            const char* refresh_min_ptr = strstr(buffer, "\"refreshInterval\":");
            if (!refresh_min_ptr) {
                refresh_min_ptr = strstr(buffer, "\"refresh_interval\":");
            }
            if (!refresh_min_ptr) {
                refresh_min_ptr = strstr(buffer, "\"refresh_min\":");
            }
            
            if (refresh_min_ptr) {
                const char* end_name = strchr(refresh_min_ptr, ':');
                if (end_name) {
                    refresh_min_ptr = end_name + 1;
                    while (*refresh_min_ptr == ' ' || *refresh_min_ptr == '"') refresh_min_ptr++;
                    int new_sleep_min = atoi(refresh_min_ptr);
                    if (new_sleep_min > 0 && new_sleep_min != sleep_minutes_) {
                        sleep_minutes_ = new_sleep_min;
                        ESP_LOGI(TAG, "[FOCUS] Updated sleep interval to %d minutes (from refreshInterval)", sleep_minutes_);
                    }
                }
            }
            
            // Track if we found always_active field in response
            bool found_always_active = false;
            
            // Check for is_always_active or always_active in response (true/false)
            if (strstr(buffer, "\"is_always_active\":true") != nullptr ||
                strstr(buffer, "\"is_always_active\": true") != nullptr ||
                strstr(buffer, "\"is_always_active\":1") != nullptr ||
                strstr(buffer, "\"is_always_active\": 1") != nullptr ||
                strstr(buffer, "\"always_active\":true") != nullptr ||
                strstr(buffer, "\"always_active\": true") != nullptr ||
                strstr(buffer, "\"always_active\":1") != nullptr ||
                strstr(buffer, "\"always_active\": 1") != nullptr) {
                server_always_active = true;
                found_always_active = true;
                ESP_LOGI(TAG, "[FOCUS] Found always_active=true in response");
            } else if (strstr(buffer, "\"is_always_active\":false") != nullptr ||
                      strstr(buffer, "\"is_always_active\": false") != nullptr ||
                      strstr(buffer, "\"is_always_active\":0") != nullptr ||
                      strstr(buffer, "\"is_always_active\": 0") != nullptr ||
                      strstr(buffer, "\"always_active\":false") != nullptr ||
                      strstr(buffer, "\"always_active\": false") != nullptr ||
                      strstr(buffer, "\"always_active\":0") != nullptr ||
                      strstr(buffer, "\"always_active\": 0") != nullptr) {
                server_always_active = false;
                found_always_active = true;
                ESP_LOGI(TAG, "[FOCUS] Found always_active=false in response");
            }
            
            // Track if we found focus_listening field in response
            bool found_focus_listening = false;
            
            // Check for is_focus_listening or focus_listening in response (true/false)
            if (strstr(buffer, "\"is_focus_listening\":true") != nullptr ||
                strstr(buffer, "\"is_focus_listening\": true") != nullptr ||
                strstr(buffer, "\"is_focus_listening\":1") != nullptr ||
                strstr(buffer, "\"is_focus_listening\": 1") != nullptr ||
                strstr(buffer, "\"focus_listening\":true") != nullptr ||
                strstr(buffer, "\"focus_listening\": true") != nullptr ||
                strstr(buffer, "\"focus_listening\":1") != nullptr ||
                strstr(buffer, "\"focus_listening\": 1") != nullptr) {
                focus_listening_enabled = true;
                found_focus_listening = true;
                ESP_LOGI(TAG, "[FOCUS] Found focus_listening=true in response");
            } else if (strstr(buffer, "\"is_focus_listening\":false") != nullptr ||
                      strstr(buffer, "\"is_focus_listening\": false") != nullptr ||
                      strstr(buffer, "\"is_focus_listening\":0") != nullptr ||
                      strstr(buffer, "\"is_focus_listening\": 0") != nullptr ||
                      strstr(buffer, "\"focus_listening\":false") != nullptr ||
                      strstr(buffer, "\"focus_listening\": false") != nullptr ||
                      strstr(buffer, "\"focus_listening\":0") != nullptr ||
                      strstr(buffer, "\"focus_listening\": 0") != nullptr) {
                focus_listening_enabled = false;
                found_focus_listening = true;
                ESP_LOGI(TAG, "[FOCUS] Found focus_listening=false in response");
            }
            
            // Update always_active based on server response
            ESP_LOGI(TAG, "[FOCUS] Server always_active: %s (found:%s), focus_listening: %s (found:%s), Local always_active: %s", 
                     server_always_active ? "true" : "false",
                     found_always_active ? "yes" : "no",
                     focus_listening_enabled ? "true" : "false",
                     found_focus_listening ? "yes" : "no",
                     always_active_ ? "ON" : "OFF");
            
            // Only update if we found at least one relevant field in the response
            if (found_always_active || found_focus_listening) {
                if (server_always_active || focus_listening_enabled) {
                    ESP_LOGI(TAG, "[FOCUS] Setting always_active to ON (from server)");
                    SetAlwaysActive(true);
                } else {
                    ESP_LOGI(TAG, "[FOCUS] Setting always_active to OFF (from server)");
                    SetAlwaysActive(false);
                }
            } else {
                ESP_LOGI(TAG, "[FOCUS] No always_active or focus_listening field found in response, keeping local state");
            }
        }
        
        free(buffer);
    } else {
        ESP_LOGW(TAG, "[FOCUS] Failed to fetch focus_listening config");
        ESP_LOGI(TAG, "[FOCUS] Local always_active: %s", always_active_ ? "ON" : "OFF");
    }
}

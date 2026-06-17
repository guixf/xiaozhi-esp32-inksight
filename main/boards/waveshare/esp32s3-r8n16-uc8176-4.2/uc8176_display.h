#ifndef UC8176_DISPLAY_H
#define UC8176_DISPLAY_H

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_http_client.h>
#include <functional>
#include <string>
#include "display.h"

// GPIO2 状态指示灯常量
#define STATUS_LED_GPIO ((gpio_num_t)2)
#define LED_MODE_OFF 0
#define LED_MODE_ON 1
#define LED_MODE_BLINK 2
#define LED_MODE_BLINK_FAST 3
#define LED_MODE_BLINK_SLOW 4
#define LED_MODE_DOUBLE_BLINK 5
#define LED_MODE_PULSE 6
// 参考 ledFeedback 的新模式
#define LED_MODE_ACK 7           // 快速闪2次 (ack)
#define LED_MODE_CONNECTING 8    // 慢速闪烁 (connecting)
#define LED_MODE_DOWNLOADING 9   // 快速闪3次 (downloading)
#define LED_MODE_SUCCESS 10      // 常亮1秒后熄灭 (success)
#define LED_MODE_FAIL 11         // 快速闪5次 (fail)
#define LED_MODE_FAVORITE 12     // 常亮2秒后熄灭 (favorite)
#define LED_MODE_PORTAL 13       // 保持常亮 (portal)

// 语音非活动阈值（毫秒）
#define VOICE_INACTIVITY_THRESHOLD_MS 5000

enum class DisplayState {
    IDLE,
    WAITING_IMAGE,
    DISPLAYING,
    PARTIAL_UPDATING
};

enum class VoiceState {
    INACTIVE,
    ACTIVE
};

typedef struct {
    uint8_t cs;
    uint8_t dc;
    uint8_t rst;
    uint8_t busy;
    uint8_t mosi;
    uint8_t scl;
    spi_host_device_t spi_host;
    int buffer_len;
} uc8176_spi_t;

class Uc8176Display : public Display {
public:
    Uc8176Display(int width, int height, uc8176_spi_t spi_data);
    ~Uc8176Display();

    void EPD_Init();
    void EPD_InitFast();
    void EPD_Clear();
    void EPD_Display(const uint8_t* image);
    void EPD_DisplayFast(const uint8_t* image);
    void EPD_DisplayPartial(const uint8_t* image, int x_start, int y_start, int x_end, int y_end);
    void EPD_Sleep();

    void clearBuffer();
    void RequestDisplayUpdate();
    void ProcessDisplayUpdate();
    bool IsDisplayBusy();
    uint8_t* GetBuffer() { return img_buf_; }
    int GetWidth() const { return width_; }
    int GetHeight() const { return height_; }

    void drawText(const char* msg, int x, int y, int scale);
    void drawPixel(int x, int y, bool black);
    void fillRect(int x, int y, int w, int h);
    int textWidth(int charCount, int scale);
    const uint8_t* getGlyph(char c);

    void smartDisplay(const uint8_t* image);
    void showSetupScreen(const char* apName, const char* regCode);
    void showAiChatStatus(const char* state, const char* detail);
    void showVoiceChatScreen();
    void showVoiceIndicator(bool footerCenter = false);
    void hideVoiceIndicator();
    void showError(const char* msg);
    void showDiagnostic(const char* line1, const char* line2, const char* line3, const char* line4);
    void showModePreview(const char* modeName);

    // Override SetStatus to detect standby state and trigger inksight fetch
    virtual void SetStatus(const char* status) override;
    virtual void ShowNotification(const char* notification, int duration_ms) override;

    void SetExternalImageCallback(std::function<void(uint8_t*)> callback) {
        external_image_callback_ = callback;
    }

    void TriggerImageUpdate();

    // Inksight image fetching
    void SetInksightConfig(const char* server);
    void FetchInksightImage();
    void SendHeartbeat(bool force = false);
    void CheckPendingRefresh();

    // Runtime mode control
    void SetAlwaysActive(bool active);
    bool IsAlwaysActive();
    void LoadAlwaysActiveFromNVS();
    int GetSleepMinutes();
    std::string GetRuntimeMode();
    void SetRuntimeModeChangedCallback(std::function<void(bool is_active)> callback);
    void SetDeepSleepCallback(std::function<void()> callback);
    void SetLiveModeTimerCallback(std::function<void(bool enable)> callback);
    void SetBatteryVoltage(int voltage_mv);
    void SetSuppressAutoImageFetch(bool suppress);

private:
    bool EnsureDeviceToken();
    void DoFetchInksightImage();
    void DoSendHeartbeat(bool force);
    static void InksightFetchTask(void* param);
    static void HeartbeatTask(void* param);
    static void PendingRefreshTask(void* param);
    void DoCheckPendingRefresh();
    bool HttpGetState(const char* url, char** out_buffer, int* out_len);
    bool readExact(esp_http_client_handle_t client, uint8_t* buf, int len, int timeout_ms);
    void FetchFocusListening();
    void PostRuntimeMode(const char* mode);

    void SaveDeviceToken(const char* token);
    void ClearDeviceToken();
    std::string LoadDeviceToken();
    std::string GetMacAddress();

    bool IsWifiConnected();
    bool IsVoiceInactive();

    virtual bool Lock(int timeout_ms = 0) override {
        if (timeout_ms <= 0) {
            return xSemaphoreTake(spi_mutex_, portMAX_DELAY) == pdTRUE;
        }
        return xSemaphoreTake(spi_mutex_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
    }
    virtual void Unlock() override {
        xSemaphoreGive(spi_mutex_);
    }

private:
    void EPD_SendCommand(uint8_t command);
    void EPD_SendData(uint8_t data);
    void EPD_SendDataArray(const uint8_t* data, int len);
    void writeBytes(const uint8_t* data, int length);
    void readBytes(uint8_t* data, int length);
    void read_busy();
    void Reset();
    void EPD_SetFullWindow(int x_start, int y_start, int x_end, int y_end);

    void drawHLine(int x0, int x1, int y);
    void drawWrappedAsciiText(const char* msg, int x, int y, int width, int scale, int maxLines);
    void drawRobotIcon(int ox, int oy, int scale);
    int computeMaxWrappedLines(int startY, int endY, int scale);
    const char* aiStatusTitle(const char* state);
    const char* aiStatusDetailFallback(const char* state);

    // LED control methods
    void InitializeStatusLed();
    void SetStatusLedMode(int mode);
    static void LedBlinkTimerCallback(void* arg);

    // Display helper methods
    void showWifiConfigScreen(const char* ap_name, const char* web_url);
    void spi_gpio_init();
    void spi_port_init();
    void SPI_SendByte(uint8_t data);
    void EPD_TurnOnDisplay();
    void EPD_TurnOnDisplayFast();
    void EPD_Reset();
    void EPD_SetFullWindow();

    int width_;
    int height_;
    uc8176_spi_t spi_data_;
    spi_device_handle_t spi_handle_;
    SemaphoreHandle_t spi_mutex_;
    uint8_t* img_buf_;
    uint8_t* red_buf_;  // Red color buffer for BWR display
    int refresh_count_;
    DisplayState display_state_;
    VoiceState voice_state_;

    // LED status indicator
    esp_timer_handle_t led_blink_timer_;
    int led_mode_;
    bool led_state_on_;
    int double_blink_count_;

    static constexpr int ROBOT_ICON_W = 24;
    static constexpr int ROBOT_ICON_H = 24;
    static constexpr int VOICE_PARTIAL_MAX = (ROBOT_ICON_W / 8 + 1) * ROBOT_ICON_H;
    uint8_t voice_icon_backup_[ROBOT_ICON_W / 8 * ROBOT_ICON_H];
    uint8_t voice_partial_buf_[VOICE_PARTIAL_MAX];
    int voice_icon_x_;
    int voice_icon_y_;

    std::function<void(uint8_t*)> external_image_callback_;

    // Inksight heartbeat
    std::string inksight_server_;
    std::string inksight_device_token_;
    bool inksight_fetch_pending_;
    TaskHandle_t inksight_task_handle_;
    uint64_t last_heartbeat_ms_;
    static constexpr uint64_t HEARTBEAT_INTERVAL_MS = 10ULL * 60ULL * 1000ULL; // 10 minutes

    // Runtime mode control
    bool always_active_;
    bool suppress_auto_image_fetch_;
    int battery_voltage_mv_;
    std::string runtime_mode_;
    int sleep_minutes_;
    bool config_fetched_;
    std::function<void(bool is_active)> runtime_mode_changed_callback_;
    std::function<void()> enter_deep_sleep_callback_;
    std::function<void(bool enable)> live_mode_timer_callback_;

    bool display_busy_;
    bool display_pending_;
    bool spi_initialized_;
    uint8_t* pending_image_;

    // Voice activity tracking
    uint64_t last_voice_activity_ms_;

    // GPIO control helpers
    inline void set_cs_0() { gpio_set_level((gpio_num_t)spi_data_.cs, 0); }
    inline void set_cs_1() { gpio_set_level((gpio_num_t)spi_data_.cs, 1); }
    inline void set_dc_0() { gpio_set_level((gpio_num_t)spi_data_.dc, 0); }
    inline void set_dc_1() { gpio_set_level((gpio_num_t)spi_data_.dc, 1); }
    inline void set_rst_0() { gpio_set_level((gpio_num_t)spi_data_.rst, 0); }
    inline void set_rst_1() { gpio_set_level((gpio_num_t)spi_data_.rst, 1); }
};

#endif  // SSD1683_DISPLAY_H

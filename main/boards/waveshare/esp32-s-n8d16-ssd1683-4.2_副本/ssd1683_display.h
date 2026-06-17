#ifndef SSD1683_DISPLAY_H
#define SSD1683_DISPLAY_H

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>
#include <string>
#include "display.h"

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
} ssd1683_spi_t;

class Ssd1683Display : public Display {
public:
    Ssd1683Display(int width, int height, ssd1683_spi_t spi_data);
    ~Ssd1683Display();

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

    void SetExternalImageCallback(std::function<void(uint8_t*)> callback) {
        external_image_callback_ = callback;
    }

    void TriggerImageUpdate();

    // Inksight image fetching
    void SetInksightConfig(const char* server);
    void SetBatteryVoltage(int voltage_mv);
    void SetRuntimeModeChangedCallback(std::function<void(bool is_active)> callback);
    void SetDeepSleepCallback(std::function<void()> callback);
    void FetchInksightImage();
    void SendHeartbeat(bool force = false);
    void PostRuntimeMode(const char* mode = nullptr);
    int GetSleepMinutes();
    std::string GetRuntimeMode();
    void SetAlwaysActive(bool active);
    bool IsAlwaysActive();
    void LoadAlwaysActiveFromNVS();
    void FetchFocusListening();
    void SetLiveModeTimerCallback(std::function<void(bool enable)> callback);
    void CheckPendingRefresh();
    
private:
    bool EnsureDeviceToken();
    void DoFetchInksightImage();
    void DoSendHeartbeat(bool force);
    static void InksightFetchTask(void* param);
    void DoCheckPendingRefresh();
    static void HeartbeatTask(void* param);

    void SaveDeviceToken(const char* token);
    void ClearDeviceToken();
    std::string LoadDeviceToken();
    std::string GetMacAddress();
    
    bool IsWifiConnected();
    bool IsVoiceInactive();

    virtual bool Lock(int timeout_ms = 0) override {
        return true;
    }
    virtual void Unlock() override {
    }

private:
    void EPD_SendCommand(uint8_t command);
    void EPD_SendData(uint8_t data);
    void writeBytes(const uint8_t* data, int length);
    void read_busy();
    void EPD_Reset();
    void EPD_SetFullWindow();

    void set_cs_1() { gpio_set_level((gpio_num_t)spi_data_.cs, 1); }
    void set_cs_0() { gpio_set_level((gpio_num_t)spi_data_.cs, 0); }
    void set_dc_1() { gpio_set_level((gpio_num_t)spi_data_.dc, 1); }
    void set_dc_0() { gpio_set_level((gpio_num_t)spi_data_.dc, 0); }
    void set_rst_1() { gpio_set_level((gpio_num_t)spi_data_.rst, 1); }
    void set_rst_0() { gpio_set_level((gpio_num_t)spi_data_.rst, 0); }

    void spi_gpio_init();
    void spi_port_init();
    void SPI_SendByte(uint8_t data);
    void EPD_TurnOnDisplay();
    void EPD_TurnOnDisplayFast();

    void drawHLine(int x0, int x1, int y);
    void drawWrappedAsciiText(const char* msg, int x, int y, int width, int scale, int maxLines);
    void drawRobotIcon(int ox, int oy, int scale);
    int computeMaxWrappedLines(int startY, int endY, int scale);
    const char* aiStatusTitle(const char* state);
    const char* aiStatusDetailFallback(const char* state);

    int width_;
    int height_;
    ssd1683_spi_t spi_data_;
    spi_device_handle_t spi_handle_;
    uint8_t* img_buf_;
    int refresh_count_;
    DisplayState display_state_;
    VoiceState voice_state_;

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
    static constexpr uint64_t HEARTBEAT_INTERVAL_MS = 1ULL * 60ULL * 1000ULL; // 1 minute for testing
    int battery_voltage_mv_ = 3700;
    
    // Runtime mode (active/interval)
    std::string runtime_mode_ = "active";
    bool always_active_ = false;  // 本地"始终保持活跃"配置
    int sleep_minutes_ = 10;
    std::function<void(bool is_active)> runtime_mode_changed_callback_;
    std::function<void()> enter_deep_sleep_callback_;
    std::function<void(bool enable)> live_mode_timer_callback_;  // 控制 live mode 定时器

    bool display_busy_;
    bool display_pending_;
    bool spi_initialized_;
    uint8_t* pending_image_;
    uint64_t last_voice_activity_ms_;
    static constexpr uint64_t VOICE_INACTIVITY_THRESHOLD_MS = 5000ULL; // 5秒无语音活动

    bool HttpGetState(const char* url, char** out_buffer, int* out_len);
    static void PendingRefreshTask(void* param);
};

#endif  // SSD1683_DISPLAY_H

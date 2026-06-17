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
    void FetchInksightImage();
    void SendHeartbeat(bool force = false);

private:
    bool EnsureDeviceToken();
    void DoFetchInksightImage();
    void DoSendHeartbeat(bool force);
    static void InksightFetchTask(void* param);
    static void HeartbeatTask(void* param);

    void SaveDeviceToken(const char* token);
    void ClearDeviceToken();
    std::string LoadDeviceToken();
    std::string GetMacAddress();
    
    bool IsWifiConnected();

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

    int width_;
    int height_;
    ssd1683_spi_t spi_data_;
    spi_device_handle_t spi_handle_;
    SemaphoreHandle_t spi_mutex_;
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
    static constexpr uint64_t HEARTBEAT_INTERVAL_MS = 10ULL * 60ULL * 1000ULL; // 10 minutes

    bool display_busy_;
    bool display_pending_;
    bool spi_initialized_;
    uint8_t* pending_image_;
};

#endif  // SSD1683_DISPLAY_H

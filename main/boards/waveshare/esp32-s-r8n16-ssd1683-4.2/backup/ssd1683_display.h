#ifndef __SSD1683_DISPLAY_H__
#define __SSD1683_DISPLAY_H__

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "display.h"

typedef struct {
    uint8_t cs;
    uint8_t dc;
    uint8_t rst;
    uint8_t busy;
    uint8_t mosi;
    uint8_t scl;
    int spi_host;
    int buffer_len;
} ssd1683_spi_t;

enum class DisplayState {
    IDLE,
    SENDING_DATA,
    WAITING_FOR_BUSY,
    COMPLETED
};

class Ssd1683Display : public Display {
public:
    Ssd1683Display(int width, int height, ssd1683_spi_t spi_data);
    ~Ssd1683Display();

    virtual void SetEmotion(const char* emotion) override;
    virtual void SetStatus(const char* status) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override;
    virtual void UpdateStatusBar(bool update_all = false) override;
    virtual void SetPowerSaveMode(bool on) override;
    virtual void SetTheme(Theme* theme) override;

    void EPD_Init();
    void EPD_InitFast();
    void EPD_Clear();
    void EPD_Display(const uint8_t* image);
    void EPD_DisplayFast(const uint8_t* image);
    void EPD_Sleep();

    void RequestDisplayUpdate();
    void ProcessDisplayUpdate();
    bool IsDisplayBusy();

    uint8_t* GetBuffer() { return img_buf_; }
    int GetWidth() const { return width_; }
    int GetHeight() const { return height_; }
    void clearBuffer();
    void drawText(const char* msg, int x, int y, int scale);

private:
    const ssd1683_spi_t spi_data_;
    const int width_;
    const int height_;
    spi_device_handle_t spi_;
    uint8_t* img_buf_;
    int refresh_count_;
    bool display_pending_;
    DisplayState display_state_;
    uint8_t* pending_image_;
    bool use_full_refresh_;

    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    void spi_gpio_init();
    void spi_port_init();
    void read_busy();

    void set_cs_1() { gpio_set_level((gpio_num_t)spi_data_.cs, 1); }
    void set_cs_0() { gpio_set_level((gpio_num_t)spi_data_.cs, 0); }
    void set_dc_1() { gpio_set_level((gpio_num_t)spi_data_.dc, 1); }
    void set_dc_0() { gpio_set_level((gpio_num_t)spi_data_.dc, 0); }
    void set_rst_1() { gpio_set_level((gpio_num_t)spi_data_.rst, 1); }
    void set_rst_0() { gpio_set_level((gpio_num_t)spi_data_.rst, 0); }

    void SPI_SendByte(uint8_t data);
    void EPD_SendData(uint8_t data);
    void EPD_SendCommand(uint8_t command);
    void writeBytes(const uint8_t* buffer, int len);
    void EPD_TurnOnDisplay();
    void EPD_TurnOnDisplayFast();
    void EPD_Reset();
    void EPD_SetFullWindow();

    void drawPixel(int x, int y, bool black);
    void fillRect(int x, int y, int w, int h);
    int textWidth(int charCount, int scale);
    const uint8_t* getGlyph(char c);
};

#endif
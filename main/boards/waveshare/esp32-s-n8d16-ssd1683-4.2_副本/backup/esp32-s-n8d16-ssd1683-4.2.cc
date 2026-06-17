#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_log.h>
#include <stdio.h>
#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "mcp_server.h"
#include "ssd1683_display.h"
#include "wifi_board.h"
#include "assets/lang_config.h"

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <nvs_flash.h>

#define TAG "esp32_s_n8d16_ssd1683_42"
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12
#define BOOT_BUTTON_HOLD_MS 10000
#define DISPLAY_REFRESH_INTERVAL_US ((uint64_t)60 * 60 * 1000 * 1000)

class Esp32SN8d16Ssd168342 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Button ai_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    Ssd1683Display* display_;
    adc_oneshot_unit_handle_t adc1_handle_;
    adc_cali_handle_t adc1_cali_handle_;
    bool adc_calibrated_;
    esp_timer_handle_t battery_timer_;
    esp_timer_handle_t refresh_timer_;
    int last_battery_voltage_;
    int last_battery_percentage_;
    uint64_t last_refresh_time_;
    bool wifi_connected_;
    bool in_deep_sleep_;

    static void BatteryTimerCallback(void* arg) {
        static uint8_t timer_count_ = 0;
        timer_count_++;
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        board->display_->ProcessDisplayUpdate();
        if (timer_count_ >= 10) {
            board->UpdateBatteryStatus();
            timer_count_ = 0;
        }
    }

    static void RefreshTimerCallback(void* arg) {
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        board->ScheduleDisplayRefresh();
    }

    void UpdateBatteryStatus() {
        last_battery_voltage_ = ReadBatteryVoltage();
        last_battery_percentage_ = CalculateBatteryPercentage(last_battery_voltage_);

        if (adc_calibrated_) {
            ESP_LOGI(TAG, "Battery status - Voltage: %d mV, Level: %d%%",
                     last_battery_voltage_, last_battery_percentage_);
        }
    }

    void ScheduleDisplayRefresh() {
        if (wifi_connected_) {
            display_->SetStatus("WiFi OK");
        }
    }

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {};
        i2c_bus_cfg.i2c_port = (i2c_port_t)0;
        i2c_bus_cfg.sda_io_num = AUDIO_CODEC_I2C_SDA_PIN;
        i2c_bus_cfg.scl_io_num = AUDIO_CODEC_I2C_SCL_PIN;
        i2c_bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt = 7;
        i2c_bus_cfg.intr_priority = 0;
        i2c_bus_cfg.trans_queue_depth = 0;
        i2c_bus_cfg.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeButtons() {
        boot_button_.OnLongPress([this]() {
            EnterWifiConfigMode();
        });
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        ai_button_.OnClick([this]() {
            HandleAiButton();
        });

        ai_button_.OnLongPress([this]() {
            HandleAiButton();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    void HandleAiButton() {
        if (in_deep_sleep_) {
            WakeFromDeepSleep();
            return;
        }
        
        auto& app = Application::GetInstance();
        app.ToggleChatState();
        
        display_->SetChatMessage("AI", "CHAT MODE");
    }

    void WakeFromDeepSleep() {
        in_deep_sleep_ = false;
        display_->EPD_Init();
        
        auto& app = Application::GetInstance();
        app.ToggleChatState();
        
        display_->SetChatMessage("AI", "WAKE UP");
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.disp.network", "重新配网", PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               EnterWifiConfigMode();
                               return true;
                           });
    }

    void InitializeDisplay() {
        ssd1683_spi_t epd_spi_data = {};
        epd_spi_data.cs = EPD_CS_PIN;
        epd_spi_data.dc = EPD_DC_PIN;
        epd_spi_data.rst = EPD_RST_PIN;
        epd_spi_data.busy = EPD_BUSY_PIN;
        epd_spi_data.mosi = EPD_MOSI_PIN;
        epd_spi_data.scl = EPD_SCK_PIN;
        epd_spi_data.spi_host = EPD_SPI_NUM;
        epd_spi_data.buffer_len = EXAMPLE_LCD_WIDTH * EXAMPLE_LCD_HEIGHT / 8;
        display_ = new Ssd1683Display(EXAMPLE_LCD_WIDTH, EXAMPLE_LCD_HEIGHT, epd_spi_data);
    }

    void InitializeBatteryAdc() {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle_));

        adc_oneshot_chan_cfg_t config = {
            .atten = BATTERY_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle_, BATTERY_ADC_CHANNEL, &config));

        adc_cali_handle_t handle = NULL;
        esp_err_t ret = ESP_FAIL;
        adc_calibrated_ = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (!adc_calibrated_) {
            ESP_LOGI(TAG, "Use curve fitting calibration scheme");
            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = ADC_UNIT_1,
                .chan = BATTERY_ADC_CHANNEL,
                .atten = BATTERY_ADC_ATTEN,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                adc_calibrated_ = true;
            }
        }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if (!adc_calibrated_) {
            ESP_LOGI(TAG, "Use line fitting calibration scheme");
            adc_cali_line_fitting_config_t cali_config = {
                .unit_id = ADC_UNIT_1,
                .atten = BATTERY_ADC_ATTEN,
                .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
            if (ret == ESP_OK) {
                adc_calibrated_ = true;
            }
        }
#endif

        adc1_cali_handle_ = handle;
    }

    void InitializeTimers() {
        esp_timer_create_args_t battery_timer_args = {
            .callback = &BatteryTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "battery_timer",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&battery_timer_args, &battery_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(battery_timer_, 1000 * 1000));

        esp_timer_create_args_t refresh_timer_args = {
            .callback = &RefreshTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "refresh_timer",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&refresh_timer_args, &refresh_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(refresh_timer_, DISPLAY_REFRESH_INTERVAL_US));
    }

    void InitializeWakeupSource() {
        gpio_wakeup_enable((gpio_num_t)AI_BUTTON_GPIO, GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
    }

    int ReadBatteryVoltage() {
        int adc_raw = 0;
        long long sum = 0;
        int voltage = 0;
        for (int i = 0; i < 10; i++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle_, BATTERY_ADC_CHANNEL, &adc_raw));
            sum += adc_raw;
        }
        adc_raw = sum / 10;

        if (adc_calibrated_) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle_, adc_raw, &voltage));
        }

        return voltage * 3;
    }

    int CalculateBatteryPercentage(int voltage) {
        if (!adc_calibrated_ || voltage <= 0) {
            return -1;
        }

        const int min_voltage = 3000;
        const int max_voltage = 4200;

        if (voltage < min_voltage) {
            return 0;
        } else if (voltage > max_voltage) {
            return 100;
        } else {
            return (voltage - min_voltage) * 100 / (max_voltage - min_voltage);
        }
    }

    std::string GetRegistrationCode() {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
        if (err != ESP_OK) {
            return "N/A";
        }

        char reg_code[32] = {0};
        size_t len = sizeof(reg_code);
        err = nvs_get_str(nvs_handle, "registration_code", reg_code, &len);
        nvs_close(nvs_handle);

        if (err != ESP_OK) {
            return "N/A";
        }

        return std::string(reg_code);
    }

    void ShowWifiConfigScreen() {
        std::string reg_code = GetRegistrationCode();
        
        display_->clearBuffer();
        
        int y = 20;
        
        display_->drawText("SETUP WIFI", 50, y, 3);
        y += 40;
        
        display_->drawText("REG CODE:", 20, y, 2);
        y += 25;
        
        display_->drawText(reg_code.c_str(), 20, y, 2);
        y += 30;
        
        display_->drawText("CONNECT TO AP", 20, y, 2);
        y += 25;
        
        display_->drawText("OPEN BROWSER", 20, y, 2);
        
        display_->EPD_DisplayFast(display_->GetBuffer());
    }

void OnNetworkEvent(NetworkEvent event, const std::string& data = "") {
        WifiBoard::OnNetworkEvent(event, data);
        
        switch (event) {
            case NetworkEvent::Connected:
                wifi_connected_ = true;
                display_->SetStatus("WiFi: OK");
                break;
            case NetworkEvent::Disconnected:
                wifi_connected_ = false;
                display_->SetStatus("WiFi: DISCONNECTED");
                break;
            case NetworkEvent::Connecting:
                display_->SetStatus("WiFi: CONNECTING...");
                break;
            case NetworkEvent::WifiConfigModeEnter:
                ShowWifiConfigScreen();
                break;
            default:
                break;
        }
    }

public:
    Esp32SN8d16Ssd168342()
        : boot_button_(BOOT_BUTTON_GPIO, true, BOOT_BUTTON_HOLD_MS),
          ai_button_(AI_BUTTON_GPIO, true),
          volume_up_button_(VOLUME_UP_BUTTON_GPIO, true),
          volume_down_button_(VOLUME_DOWN_BUTTON_GPIO, true),
          last_battery_voltage_(0),
          last_battery_percentage_(-1),
          wifi_connected_(false),
          in_deep_sleep_(false) {
        ESP_LOGI(TAG, "Initializing ESP32-S-N8D16-SSD1683-4.2 Board");

        InitializeI2c();
        InitializeButtons();
        InitializeTools();
        InitializeDisplay();
        InitializeBatteryAdc();
        InitializeTimers();
        InitializeWakeupSource();

        UpdateBatteryStatus();

        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
            uint32_t gpio_status = esp_sleep_get_ext1_wakeup_status();
            if (gpio_status & (1ULL << AI_BUTTON_GPIO)) {
                ESP_LOGI(TAG, "Woken by AI button");
                in_deep_sleep_ = true;
            }
        }
    }

    ~Esp32SN8d16Ssd168342() {
        esp_timer_stop(battery_timer_);
        esp_timer_delete(battery_timer_);
        esp_timer_stop(refresh_timer_);
        esp_timer_delete(refresh_timer_);

        ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle_));

        if (adc_calibrated_) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
            ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc1_cali_handle_));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
            ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(adc1_cali_handle_));
#endif
        }
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    int GetBatteryVoltage() {
        return last_battery_voltage_;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = last_battery_percentage_;
        charging = false;
        discharging = true;
        return true;
    }

    void EnterDeepSleep() {
        display_->EPD_Sleep();
        in_deep_sleep_ = true;
        
        esp_deep_sleep_start();
    }
};

DECLARE_BOARD(Esp32SN8d16Ssd168342);
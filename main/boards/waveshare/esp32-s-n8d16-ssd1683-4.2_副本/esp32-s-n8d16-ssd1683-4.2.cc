#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_log.h>
#include <stdio.h>
#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "mcp_server.h"
#include "settings.h"
#include "ssd1683_display.h"
#include "wifi_board.h"
#include "wifi_manager.h"
#include "assets/lang_config.h"

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
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
    esp_timer_handle_t heartbeat_timer_;
    esp_timer_handle_t live_mode_timer_;
    int last_battery_voltage_;
    int last_battery_percentage_;
    uint64_t last_refresh_time_;
    bool wifi_connected_;
    bool in_deep_sleep_;
    bool wakeup_fetch_done_;  // 标记唤醒后是否已获取图片
    NetworkEventCallback saved_network_callback_;

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

    static void HeartbeatTimerCallback(void* arg) {
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        board->display_->SendHeartbeat();
    }

    static void LiveModeTimerCallback(void* arg) {
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        board->display_->CheckPendingRefresh();
    }

    void HandleNetworkEventCallback(NetworkEvent event, const std::string& data = "") {
        // 调用父类的处理
        WifiBoard::OnNetworkEvent(event, data);
        
        switch (event) {
            case NetworkEvent::Connected:
                wifi_connected_ = true;
                display_->showAiChatStatus("IDLE", "WiFi OK");
                ESP_LOGI(TAG, "[RUNTIME] Network connected event received");
                // WiFi 连接成功后，先发送心跳让服务器知道设备在线
                ESP_LOGI(TAG, "[HEARTBEAT] WiFi connected, sending heartbeat first...");
                display_->SendHeartbeat(true);
                // 然后获取 Inksight 图片
                ESP_LOGI(TAG, "[INKSIGHT] Fetching image after heartbeat...");
                display_->FetchInksightImage();
                break;
            case NetworkEvent::Disconnected:
                wifi_connected_ = false;
                display_->showAiChatStatus("IDLE", "WiFi DISCONNECTED");
                break;
            case NetworkEvent::Connecting:
                display_->showAiChatStatus("CONNECTING", "WS CONNECT");
                break;
            case NetworkEvent::Scanning:
                display_->showAiChatStatus("CONNECTING", "SCANNING...");
                break;
            case NetworkEvent::WifiConfigModeEnter:
                ShowWifiConfigScreen();
                break;
            case NetworkEvent::WifiConfigModeExit:
                break;
            default:
                break;
        }
        
        // 调用保存的回调
        if (saved_network_callback_) {
            saved_network_callback_(event, data);
        }
    }

    void UpdateBatteryStatus() {
        last_battery_voltage_ = ReadBatteryVoltage();
        last_battery_percentage_ = CalculateBatteryPercentage(last_battery_voltage_);

        if (adc_calibrated_) {
            ESP_LOGI(TAG, "Battery status - Voltage: %d mV, Level: %d%%",
                     last_battery_voltage_, last_battery_percentage_);
        }
        
        // 把电池电压传给display
        display_->SetBatteryVoltage(last_battery_voltage_);
    }

    void ScheduleDisplayRefresh() {
        if (wifi_connected_) {
            display_->showAiChatStatus("IDLE", "WiFi OK");
        }
    }

    void CheckAndInitializeInksight() {
        auto& wifi_manager = WifiManager::GetInstance();
        if (wifi_manager.IsConnected()) {
            ESP_LOGI(TAG, "[INKSIGHT] WiFi already connected, initializing...");
            wifi_connected_ = true;
            
            if (!in_deep_sleep_) {
                // 首次启动：刷新屏幕，显示 WiFi OK
                ESP_LOGI(TAG, "[INKSIGHT] First boot, refreshing screen");
                display_->showAiChatStatus("IDLE", "WiFi OK");
                // 获取 Inksight 图片
                display_->FetchInksightImage();
            } else {
                // 唤醒状态：跳过所有屏幕刷新，直接开始对话
                ESP_LOGI(TAG, "[INKSIGHT] Wake from deep sleep, skipping screen refresh, starting chat directly");
            }
            
            // 立即发送一次心跳
            display_->SendHeartbeat(true);
        } else {
            ESP_LOGI(TAG, "[INKSIGHT] WiFi not connected yet, waiting for event...");
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
        
        display_->showAiChatStatus("LISTENING", "READY");
    }

    void WakeFromDeepSleep() {
        in_deep_sleep_ = false;
        wakeup_fetch_done_ = false;  // 重置标记，对话结束后需要获取图片
        
        auto& app = Application::GetInstance();
        app.ToggleChatState();
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

        display_->EPD_Init();

        // 设置 Inksight 服务配置 - 使用独立的 inksight 服务器地址
        Settings websocket_settings("websocket", false);
        std::string inksight_url = websocket_settings.GetString("inksight_url");
        
        // 如果没有配置独立的 inksight_url，使用默认的 web.inksight.site
        if (inksight_url.empty()) {
            inksight_url = "https://web.inksight.site";
            ESP_LOGI(TAG, "[INKSIGHT] Using default server: %s", inksight_url.c_str());
        }
        
        if (!inksight_url.empty()) {
            display_->SetInksightConfig(inksight_url.c_str());
            ESP_LOGI(TAG, "[INKSIGHT] Configured server: %s", inksight_url.c_str());
            
            // 从 NVS 加载 always_active 配置
            display_->LoadAlwaysActiveFromNVS();
            ESP_LOGI(TAG, "[RUNTIME] Always active mode: %s", display_->IsAlwaysActive() ? "ON" : "OFF");
            
            // 设置运行时模式改变的回调
            display_->SetRuntimeModeChangedCallback([this](bool is_active) {
                ESP_LOGI(TAG, "[RUNTIME] Mode changed to: %s", is_active ? "active" : "interval");
                if (!is_active) {
                    // interval模式下，设备会在fetch完成后进入深度睡眠
                    ESP_LOGI(TAG, "[RUNTIME] Will enter deep sleep after fetch completes");
                } else {
                    // active模式下，保持活跃
                    ESP_LOGI(TAG, "[RUNTIME] Keeping device active");
                }
            });
            
            // 设置进入深度睡眠的回调
            display_->SetDeepSleepCallback([this]() {
                ESP_LOGI(TAG, "[RUNTIME] Deep sleep callback triggered");
                EnterDeepSleep();
            });
            
            // 设置 live mode 定时器回调
            display_->SetLiveModeTimerCallback([this](bool enable) {
                if (enable) {
                    StartLiveModeTimer();
                } else {
                    StopLiveModeTimer();
                }
            });
            
            // 初始化完成后立即触发一次心跳和 pending refresh 检查
            vTaskDelay(pdMS_TO_TICKS(2000));
            ESP_LOGI(TAG, "[RUNTIME] Initial runtime mode: %s", display_->GetRuntimeMode().c_str());
            // 注意：PostRuntimeMode 在 WiFi 连接成功后调用，避免网络栈未就绪
            display_->SendHeartbeat(true);
        } else {
            ESP_LOGW(TAG, "[INKSIGHT] No server URL configured");
        }
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

        // Heartbeat timer - 10 minutes
        esp_timer_create_args_t heartbeat_timer_args = {
            .callback = &HeartbeatTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "heartbeat_timer",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&heartbeat_timer_args, &heartbeat_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(heartbeat_timer_, 10ULL * 60ULL * 1000ULL * 1000ULL)); // 10 minutes

        // Live mode timer - 5 seconds (not started by default)
        esp_timer_create_args_t live_mode_timer_args = {
            .callback = &LiveModeTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "live_mode_timer",
            .skip_unhandled_events = false,
        };
        ESP_ERROR_CHECK(esp_timer_create(&live_mode_timer_args, &live_mode_timer_));
    }

    void StartLiveModeTimer() {
        if (esp_timer_is_active(live_mode_timer_)) {
            esp_timer_stop(live_mode_timer_);
        }
        ESP_LOGI(TAG, "[LIVE MODE] Starting live mode timer (5 seconds)");
        ESP_ERROR_CHECK(esp_timer_start_periodic(live_mode_timer_, 5ULL * 1000ULL * 1000ULL)); // 5 seconds
    }

    void StopLiveModeTimer() {
        if (esp_timer_is_active(live_mode_timer_)) {
            ESP_LOGI(TAG, "[LIVE MODE] Stopping live mode timer");
            esp_timer_stop(live_mode_timer_);
        }
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
        display_->showSetupScreen("XIAOZHI_XXXX", reg_code.c_str());
    }

public:
    Esp32SN8d16Ssd168342()
        : boot_button_(BOOT_BUTTON_GPIO, true, BOOT_BUTTON_HOLD_MS),
          ai_button_(AI_BUTTON_GPIO, false),  // AI 按钮是低电平有效
          volume_up_button_(VOLUME_UP_BUTTON_GPIO, true),
          volume_down_button_(VOLUME_DOWN_BUTTON_GPIO, true),
          last_battery_voltage_(0),
          last_battery_percentage_(-1),
          wifi_connected_(false),
          in_deep_sleep_(false),
          wakeup_fetch_done_(false) {
        ESP_LOGI(TAG, "Initializing ESP32-S-N8D16-SSD1683-4.2 Board");

        InitializeI2c();
        InitializeButtons();
        InitializeTools();
        InitializeDisplay();
        InitializeBatteryAdc();
        InitializeTimers();

        UpdateBatteryStatus();

        // 设置网络事件回调
        SetNetworkEventCallback([this](NetworkEvent event, const std::string& data) {
            HandleNetworkEventCallback(event, data);
        });

        esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
        ESP_LOGI(TAG, "[RUNTIME] Wakeup cause: %d", wakeup_cause);
        
        if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0 || 
            wakeup_cause == ESP_SLEEP_WAKEUP_EXT1 || 
            wakeup_cause == ESP_SLEEP_WAKEUP_TIMER) {
            if (wakeup_cause == ESP_SLEEP_WAKEUP_TIMER) {
                ESP_LOGI(TAG, "[RUNTIME] Woken by timer (wake from deep sleep)");
            } else {
                ESP_LOGI(TAG, "[RUNTIME] Woken by GPIO/EXT (wake from deep sleep)");
            }
            in_deep_sleep_ = true;
            ESP_LOGI(TAG, "[RUNTIME] Calling WakeFromDeepSleep to start chat...");
            WakeFromDeepSleep();
        } else {
            ESP_LOGI(TAG, "[RUNTIME] First boot (not wake from deep sleep)");
            in_deep_sleep_ = false;
            
            // 如果是第一次启动（非唤醒），显示初始页面
            ESP_LOGI(TAG, "[RUNTIME] First boot, showing initial screen");
            display_->showAiChatStatus("IDLE", "INITIALIZING...");
        }
        
        // 检查 WiFi 是否已连接（防止回调设置前已连接）
        ESP_LOGI(TAG, "[INKSIGHT] Scheduling CheckAndInitializeInksight...");
        Application::GetInstance().Schedule([this]() {
            ESP_LOGI(TAG, "[INKSIGHT] CheckAndInitializeInksight scheduled task running");
            CheckAndInitializeInksight();
        });
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
    
    bool IsWakeFromDeepSleep() const { return in_deep_sleep_; }
    bool IsWakeupFetchDone() const { return wakeup_fetch_done_; }
    
    void FetchImageAfterWake() {
        if (!wakeup_fetch_done_) {
            wakeup_fetch_done_ = true;
            display_->FetchInksightImage();
            ESP_LOGI(TAG, "[INKSIGHT] Wake from deep sleep, image fetched after chat");
        }
    }
    
    void OnDeviceIdle() override {
        if (!wakeup_fetch_done_) {
            ESP_LOGI(TAG, "[INKSIGHT] Wake from deep sleep, fetching image after chat ended");
            wakeup_fetch_done_ = true;
            display_->FetchInksightImage();
        }
    }
    
    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = last_battery_percentage_;
        charging = false;
        discharging = true;
        return true;
    }

    void EnterDeepSleep() {
        ESP_LOGI(TAG, "[RUNTIME] Entering deep sleep...");
        
        // 检查 always_active 模式
        if (display_->IsAlwaysActive()) {
            ESP_LOGI(TAG, "[RUNTIME] Always active mode is ON, skipping deep sleep");
            return;
        }
        
        // 检查设备状态，如果正在活跃对话（listening/speaking），不进入休眠
        DeviceState state = Application::GetInstance().GetDeviceState();
        ESP_LOGI(TAG, "[RUNTIME] Current device state: %d", state);
        if (state == kDeviceStateListening || state == kDeviceStateSpeaking) {
            ESP_LOGI(TAG, "[RUNTIME] Device is active (state=%d), skipping deep sleep", state);
            return;
        }
        
        display_->EPD_Sleep();
        in_deep_sleep_ = true;
        
        // 获取睡眠时长（从display获取）
        int sleep_minutes = display_->GetSleepMinutes();
        uint64_t sleep_us = (uint64_t)sleep_minutes * 60 * 1000 * 1000;
        ESP_LOGI(TAG, "[RUNTIME] Deep sleep duration: %d minutes (%llu us)", sleep_minutes, sleep_us);
        
        // 断开 WiFi 连接前，先等待一下让其他任务完成
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 断开 WiFi 连接
        esp_wifi_disconnect();
        esp_wifi_stop();
        
        // 等待 WiFi 完全停止
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // 关键：禁用所有唤醒源（清除之前的配置）
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        
        // 配置RTC外设电源域，确保唤醒有效
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        
        // 配置AI按钮GPIO为输入模式，启用内部上拉
        gpio_config_t ai_button_config = {};
        ai_button_config.pin_bit_mask = (1ULL << AI_BUTTON_GPIO);
        ai_button_config.mode = GPIO_MODE_INPUT;
        ai_button_config.pull_up_en = GPIO_PULLUP_ENABLE;
        ai_button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ai_button_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&ai_button_config);
        
        // 读取AI按钮状态，检查是否被按下
        int ai_button_state = gpio_get_level((gpio_num_t)AI_BUTTON_GPIO);
        ESP_LOGI(TAG, "[RUNTIME] AI button state before sleep: %s (GPIO%d)", 
                 ai_button_state ? "HIGH" : "LOW", AI_BUTTON_GPIO);
        
        // 如果按钮被按下（LOW），不进入深度睡眠，避免立即被唤醒
        if (!ai_button_state) {
            ESP_LOGW(TAG, "[RUNTIME] AI button is pressed, skipping deep sleep");
            in_deep_sleep_ = false;
            return;
        }
        
        // 配置AI按钮GPIO为EXT1低电平唤醒
        ESP_ERROR_CHECK(rtc_gpio_init((gpio_num_t)AI_BUTTON_GPIO));
        ESP_ERROR_CHECK(rtc_gpio_pullup_en((gpio_num_t)AI_BUTTON_GPIO));  // 启用上拉
        ESP_ERROR_CHECK(rtc_gpio_pulldown_dis((gpio_num_t)AI_BUTTON_GPIO));  // 禁用下拉
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(1ULL << AI_BUTTON_GPIO, ESP_EXT1_WAKEUP_ALL_LOW)); // 低电平唤醒
        ESP_LOGI(TAG, "[RUNTIME] AI button wakeup enabled (GPIO%d, EXT1, ALL LOW)", AI_BUTTON_GPIO);
        
        // 配置定时器唤醒
        esp_sleep_enable_timer_wakeup(sleep_us);
        ESP_LOGI(TAG, "[RUNTIME] Timer wakeup enabled");
        ESP_LOGI(TAG, "[RUNTIME] Now going to deep sleep now...");
        esp_deep_sleep_start();
    }
};

DECLARE_BOARD(Esp32SN8d16Ssd168342);
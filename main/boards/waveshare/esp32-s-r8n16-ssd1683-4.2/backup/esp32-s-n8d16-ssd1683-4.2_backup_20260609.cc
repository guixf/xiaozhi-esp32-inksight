#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_log.h>
#include <esp_wifi.h>
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

// NVS键名定义
#define NVS_WAKEUP_NAMESPACE "wakeup"
#define NVS_KEY_BUTTON_FLAG "btn_wake"
#define NVS_KEY_TIMER_FLAG "timer_wake"
#define NVS_KEY_MARKER "marker"

#define WAKEUP_MARKER_VALID 0x5AA5
#define WAKEUP_MARKER_INVALID 0x0000

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
    bool wakeup_start_chat_;  // 标记唤醒后是否需要启动对话
    bool delayed_wakeup_;     // 标记是否需要延迟唤醒处理
    NetworkEventCallback saved_network_callback_;

    static void BatteryTimerCallback(void* arg) {
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        board->display_->ProcessDisplayUpdate();
    }

    static void RefreshTimerCallback(void* arg) {
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        board->ScheduleDisplayRefresh();
    }

    static void HeartbeatTimerCallback(void* arg) {
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        ESP_LOGI(TAG, "[BATTERY] Reading battery voltage before heartbeat timer...");
        board->UpdateBatteryStatus();
        board->display_->SendHeartbeat();
    }

    static void LiveModeTimerCallback(void* arg) {
        Esp32SN8d16Ssd168342* board = static_cast<Esp32SN8d16Ssd168342*>(arg);
        board->display_->CheckPendingRefresh();
    }

    void HandleNetworkEventCallback(NetworkEvent event, const std::string& data = "") {
        // 注意：不要调用 WifiBoard::OnNetworkEvent，因为会造成递归调用
        // WifiBoard::OnNetworkEvent 已经在调用这个回调之前执行过了
        
        ESP_LOGI(TAG, "[RUNTIME] HandleNetworkEventCallback: event=%d, in_deep_sleep_=%d, wakeup_start_chat_=%d", 
                 (int)event, in_deep_sleep_, wakeup_start_chat_);
        
        switch (event) {
            case NetworkEvent::Connected:
                wifi_connected_ = true;
                display_->showAiChatStatus("IDLE", "WiFi OK");
                ESP_LOGI(TAG, "[RUNTIME] Network connected event received, in_deep_sleep_=%d, wakeup_start_chat_=%d", 
                         in_deep_sleep_, wakeup_start_chat_);
                
                // 关键：只有按键唤醒（wakeup_start_chat_ 为 true 时才启动对话
                if (in_deep_sleep_ && wakeup_start_chat_) {
                    // 按键唤醒：WiFi连接后立即启动对话
                    ESP_LOGI(TAG, "[RUNTIME] Button wakeup, WiFi connected, starting chat NOW");
                    in_deep_sleep_ = false;
                    wakeup_start_chat_ = false;  // 清除标志
                    Application::GetInstance().ToggleChatState();
                } else {
                    // 首次启动或定时器唤醒：先检测电池电量，然后发送心跳和获取图片
                    ESP_LOGI(TAG, "[BATTERY] Reading battery voltage before heartbeat...");
                    UpdateBatteryStatus();
                    ESP_LOGI(TAG, "[HEARTBEAT] WiFi connected, sending heartbeat first...");
                    display_->SendHeartbeat(true);
                    ESP_LOGI(TAG, "[INKSIGHT] Fetching image after heartbeat...");
                    display_->FetchInksightImage();
                }
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

    void CheckAndInitializeInksight(bool is_wake_from_deep_sleep) {
        auto& wifi_manager = WifiManager::GetInstance();
        if (wifi_manager.IsConnected()) {
            ESP_LOGI(TAG, "[INKSIGHT] WiFi already connected, initializing...");
            wifi_connected_ = true;
            
            if (!is_wake_from_deep_sleep) {
                // 首次启动：刷新屏幕，显示 WiFi OK，获取图片
                ESP_LOGI(TAG, "[INKSIGHT] First boot, refreshing screen");
                display_->showAiChatStatus("IDLE", "WiFi OK");
                display_->SendHeartbeat(true);
                display_->FetchInksightImage();
            } else {
                // 唤醒状态：不获取图片，等待对话结束后在 OnDeviceIdle 中获取
                ESP_LOGI(TAG, "[INKSIGHT] Wake from deep sleep, skipping image fetch (will fetch after chat ends)");
                // wakeup_fetch_done_ 已经在 WakeFromDeepSleep() 中设置为 false
                // 对话结束后 OnDeviceIdle() 会检查并获取图片
            }
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
            // 检查display_是否已初始化，避免崩溃
            if (display_) {
                WakeFromDeepSleep();
            } else {
                ESP_LOGW(TAG, "[RUNTIME] HandleAiButton: display_ not initialized yet, skipping wake from deep sleep");
            }
            return;
        }
        
        auto& app = Application::GetInstance();
        app.ToggleChatState();
        
        display_->showAiChatStatus("LISTENING", "READY");
    }

    void WakeFromDeepSleep() {
        wakeup_fetch_done_ = false;  // 重置标记，对话结束后需要获取图片
        display_->SetSuppressAutoImageFetch(true);  // 抑制自动获取图片
        
        // 尝试启动对话，但可能在WiFi未连接时无法启动
        auto& app = Application::GetInstance();
        app.ToggleChatState();
        
        // 注意：不要在这里设置 in_deep_sleep_ = false
        // 让 HandleNetworkEventCallback 来处理（WiFi连接后再设置为false）
        
        // 清除NVS唤醒标志（只有成功执行唤醒处理后才清除）
        ClearNvsWakeupFlags();
    }
    
    void ClearNvsWakeupFlags() {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open(NVS_WAKEUP_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if (err == ESP_OK) {
            nvs_set_i32(nvs_handle, NVS_KEY_MARKER, WAKEUP_MARKER_INVALID);
            nvs_set_i32(nvs_handle, NVS_KEY_BUTTON_FLAG, 0);
            nvs_set_i32(nvs_handle, NVS_KEY_TIMER_FLAG, 0);
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
            ESP_LOGI(TAG, "[RUNTIME] NVS wakeup flags cleared");
        } else {
            ESP_LOGW(TAG, "[RUNTIME] Failed to clear NVS wakeup flags: %s", esp_err_to_name(err));
        }
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
          wakeup_fetch_done_(false),
          wakeup_start_chat_(false),
          delayed_wakeup_(false) {
        // 第一步：获取并判断唤醒原因（放在最开头，在任何其他初始化之前）
        // 使用三重检测机制：
        // 1. esp_sleep_get_wakeup_cause() - 标准API
        // 2. esp_sleep_get_ext1_wakeup_status() - EXT1唤醒源状态（备用）
        // 3. NVS存储 - 防止DTR/RTS复位清除RTC内存
        esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
        const char* wakeup_name = "UNKNOWN";
        bool is_gpio_wakeup = false;
        
        switch(wakeup_cause) {
            case ESP_SLEEP_WAKEUP_UNDEFINED: wakeup_name = "UNDEFINED"; break;
            case ESP_SLEEP_WAKEUP_EXT0: wakeup_name = "EXT0"; is_gpio_wakeup = true; break;
            case ESP_SLEEP_WAKEUP_EXT1: wakeup_name = "EXT1"; is_gpio_wakeup = true; break;
            case ESP_SLEEP_WAKEUP_TIMER: wakeup_name = "TIMER"; break;
            case ESP_SLEEP_WAKEUP_GPIO: wakeup_name = "GPIO"; is_gpio_wakeup = true; break;
            case ESP_SLEEP_WAKEUP_UART: wakeup_name = "UART"; break;
            case ESP_SLEEP_WAKEUP_TOUCHPAD: wakeup_name = "TOUCHPAD"; break;
            case ESP_SLEEP_WAKEUP_ULP: wakeup_name = "ULP"; break;
            default: wakeup_name = "OTHER"; break;
        }
        ESP_LOGI(TAG, "[WAKEUP_DEBUG] Wakeup cause: %d (%s)", wakeup_cause, wakeup_name);
        ESP_LOGI(TAG, "[WAKEUP_DEBUG] AI_BUTTON_GPIO: %d", AI_BUTTON_GPIO);
        
        // 检查EXT1唤醒状态（用于确认按键唤醒，作为备用检测）
        bool ai_button_wakeup = false;
        uint32_t ext1_wakeup_status = esp_sleep_get_ext1_wakeup_status();
        ESP_LOGI(TAG, "[WAKEUP_DEBUG] EXT1 wakeup status: 0x%08X", ext1_wakeup_status);
        if (ext1_wakeup_status != 0 && (ext1_wakeup_status & (1ULL << AI_BUTTON_GPIO))) {
            ai_button_wakeup = true;
            ESP_LOGI(TAG, "[WAKEUP_DEBUG] AI button wakeup detected via EXT1");
        }
        
        // 读取当前AI按钮状态（调试用）
        int ai_button_level = gpio_get_level((gpio_num_t)AI_BUTTON_GPIO);
        ESP_LOGI(TAG, "[WAKEUP_DEBUG] Current AI button level: %d (HIGH=1, LOW=0)", ai_button_level);
        
        // 检查NVS中的唤醒标志（备用方案，防止DTR/RTS复位清除RTC内存）
        bool nvs_button_wakeup = false;
        bool nvs_timer_wakeup = false;
        {
            nvs_handle_t nvs_handle;
            esp_err_t err = nvs_open(NVS_WAKEUP_NAMESPACE, NVS_READONLY, &nvs_handle);
            if (err == ESP_OK) {
                int32_t marker = WAKEUP_MARKER_INVALID;
                err = nvs_get_i32(nvs_handle, NVS_KEY_MARKER, &marker);
                if (err == ESP_OK && marker == WAKEUP_MARKER_VALID) {
                    int32_t val;
                    err = nvs_get_i32(nvs_handle, NVS_KEY_BUTTON_FLAG, &val);
                    nvs_button_wakeup = (err == ESP_OK) && (val != 0);
                    err = nvs_get_i32(nvs_handle, NVS_KEY_TIMER_FLAG, &val);
                    nvs_timer_wakeup = (err == ESP_OK) && (val != 0);
                    ESP_LOGI(TAG, "[RUNTIME] NVS wakeup flags - button:%d, timer:%d", 
                             nvs_button_wakeup, nvs_timer_wakeup);
                } else {
                    ESP_LOGI(TAG, "[RUNTIME] NVS marker invalid (0x%04X)", marker);
                }
                nvs_close(nvs_handle);
            } else {
                ESP_LOGW(TAG, "[RUNTIME] Failed to open NVS namespace: %s", esp_err_to_name(err));
            }
        }
        
        ESP_LOGI(TAG, "Initializing ESP32-S-N8D16-SSD1683-4.2 Board");

        // 第二步：根据唤醒原因执行不同分支（标准做法）
        bool is_wake_from_deep_sleep = false;
        
        if (wakeup_cause == ESP_SLEEP_WAKEUP_TIMER || nvs_timer_wakeup) {
            // 定时器唤醒（优先使用esp_sleep_get_wakeup_cause，NVS标志作为备用）
            ESP_LOGI(TAG, "[RUNTIME] Wake from deep sleep: TIMER (will not start chat)");
            is_wake_from_deep_sleep = true;
            in_deep_sleep_ = true;
            wakeup_start_chat_ = false;  // 定时器唤醒不启动对话
        } else if (is_gpio_wakeup || ai_button_wakeup || nvs_button_wakeup) {
            // 按键唤醒（GPIO/EXT0/EXT1唤醒，或EXT1状态检测到按键，或NVS标志）
            ESP_LOGI(TAG, "[RUNTIME] Wake from deep sleep: BUTTON (will start chat)");
            is_wake_from_deep_sleep = true;
            in_deep_sleep_ = true;
            wakeup_start_chat_ = true;  // 按键唤醒才启动对话
            
            // 设置延迟唤醒标志，在初始化完成后处理
            delayed_wakeup_ = true;
        } else {
            // 首次启动或其他原因
            ESP_LOGI(TAG, "[RUNTIME] First boot (or unknown wakeup cause)");
            is_wake_from_deep_sleep = false;
            in_deep_sleep_ = false;
            wakeup_start_chat_ = false;
        }
        
        InitializeI2c();
        InitializeButtons();
        InitializeTools();
        InitializeDisplay();
        InitializeBatteryAdc();
        InitializeTimers();

        UpdateBatteryStatus();

        // 设置网络事件回调
        SetNetworkEventCallback([this](NetworkEvent event, const std::string& data) {
            ESP_LOGI(TAG, "[RUNTIME] SetNetworkEventCallback called, event=%d", (int)event);
            HandleNetworkEventCallback(event, data);
        });

        // 第三步：根据唤醒原因执行不同逻辑
        bool local_wakeup_start_chat = wakeup_start_chat_;  // 保存局部变量供 lambda 使用
        if (is_wake_from_deep_sleep) {
            if (local_wakeup_start_chat) {
                // 按键唤醒：启动对话（延迟处理，等待display初始化完成）
                ESP_LOGI(TAG, "[RUNTIME] Button wakeup detected, will handle after initialization");
            } else {
                // 定时器唤醒：不启动对话，直接发送心跳和获取图片
                ESP_LOGI(TAG, "[RUNTIME] Timer wakeup, sending heartbeat and fetching image...");
                // 定时器唤醒应该跟首次启动一样的逻辑
                display_->showAiChatStatus("IDLE", "INITIALIZING...");
                
                // 定时器唤醒也需要清除NVS标志
                ClearNvsWakeupFlags();
            }
        } else {
            // 首次启动
            ESP_LOGI(TAG, "[RUNTIME] First boot, showing initial screen");
            display_->showAiChatStatus("IDLE", "INITIALIZING...");
        }
        
        // 处理延迟唤醒（按键唤醒后，等待初始化完成）
        if (delayed_wakeup_ && display_) {
            ESP_LOGI(TAG, "[RUNTIME] Processing delayed wakeup...");
            WakeFromDeepSleep();
            delayed_wakeup_ = false;
        }
        
        // 检查 WiFi 是否已连接（防止回调设置前已连接）
        ESP_LOGI(TAG, "[INKSIGHT] Scheduling CheckAndInitializeInksight...");
        Application::GetInstance().Schedule([this, is_wake_from_deep_sleep, local_wakeup_start_chat]() {
            ESP_LOGI(TAG, "[INKSIGHT] CheckAndInitializeInksight scheduled task running, is_wake=%d, start_chat=%d", 
                     is_wake_from_deep_sleep, local_wakeup_start_chat);
            // 定时器唤醒和首次启动一样，is_wake_from_deep_sleep 传 false，会发送心跳和获取图片
            CheckAndInitializeInksight(is_wake_from_deep_sleep && local_wakeup_start_chat);
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
    
    bool IsWakeFromDeepSleep() override { return in_deep_sleep_; }
    bool IsWakeupFetchDone() const { return wakeup_fetch_done_; }
    
    // 检查唤醒后是否需要启动对话，并清除标志
    bool CheckAndClearWakeupStartChat() {
        bool result = wakeup_start_chat_;
        wakeup_start_chat_ = false;
        return result;
    }
    
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
            display_->SetSuppressAutoImageFetch(false);  // 重新启用自动获取图片
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
        
        // 关键：保存 WiFi 配置到 RTC 内存，实现快速恢复
        esp_err_t err = esp_wifi_restore();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "[RUNTIME] WiFi config saved to RTC for quick restore");
        } else {
            ESP_LOGI(TAG, "[RUNTIME] WiFi restore result: %s (will need full reconnect)", esp_err_to_name(err));
        }
        
        // 关键：禁用所有唤醒源（清除之前的配置）
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        
        // 配置RTC外设电源域，确保唤醒有效
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        
        // 直接使用RTC GPIO配置（关键：不要混用gpio_config和rtc_gpio_init）
        ESP_ERROR_CHECK(rtc_gpio_init((gpio_num_t)AI_BUTTON_GPIO));
        ESP_ERROR_CHECK(rtc_gpio_set_direction((gpio_num_t)AI_BUTTON_GPIO, RTC_GPIO_MODE_INPUT_ONLY));
        ESP_ERROR_CHECK(rtc_gpio_pullup_en((gpio_num_t)AI_BUTTON_GPIO));  // 启用上拉
        ESP_ERROR_CHECK(rtc_gpio_pulldown_dis((gpio_num_t)AI_BUTTON_GPIO));  // 禁用下拉
        
        // 读取AI按钮状态，检查是否被按下
        int ai_button_state = rtc_gpio_get_level((gpio_num_t)AI_BUTTON_GPIO);
        ESP_LOGI(TAG, "[RUNTIME] AI button state before sleep: %s (GPIO%d)", 
                 ai_button_state ? "HIGH" : "LOW", AI_BUTTON_GPIO);
        
        // 如果按钮被按下（LOW），不进入深度睡眠，避免立即被唤醒
        if (!ai_button_state) {
            ESP_LOGW(TAG, "[RUNTIME] AI button is pressed, skipping deep sleep");
            in_deep_sleep_ = false;
            return;
        }
        
        // 配置GPIO唤醒（使用普通GPIO唤醒，更稳定）
        gpio_wakeup_enable((gpio_num_t)AI_BUTTON_GPIO, GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        ESP_LOGI(TAG, "[RUNTIME] AI button wakeup enabled (GPIO%d, GPIO wakeup, LOW level)", AI_BUTTON_GPIO);
        
        // 配置定时器唤醒
        esp_sleep_enable_timer_wakeup(sleep_us);
        ESP_LOGI(TAG, "[RUNTIME] Timer wakeup enabled");
        
        // 设置NVS唤醒标志（作为esp_sleep_get_wakeup_cause()的备用方案，防止DTR/RTS复位）
        {
            nvs_handle_t nvs_handle;
            esp_err_t err = nvs_open(NVS_WAKEUP_NAMESPACE, NVS_READWRITE, &nvs_handle);
            if (err == ESP_OK) {
                nvs_set_i32(nvs_handle, NVS_KEY_MARKER, WAKEUP_MARKER_VALID);
                nvs_set_i32(nvs_handle, NVS_KEY_BUTTON_FLAG, 1);   // 按键唤醒标志
                nvs_set_i32(nvs_handle, NVS_KEY_TIMER_FLAG, 0);    // 定时器唤醒标志
                nvs_commit(nvs_handle);
                nvs_close(nvs_handle);
                ESP_LOGI(TAG, "[RUNTIME] NVS wakeup flags set (button=1, timer=0)");
            } else {
                ESP_LOGW(TAG, "[RUNTIME] Failed to set NVS wakeup flags: %s", esp_err_to_name(err));
            }
        }
        
        ESP_LOGI(TAG, "[RUNTIME] Going to deep sleep...");
        esp_deep_sleep_start();
    }
};

DECLARE_BOARD(Esp32SN8d16Ssd168342);

#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_now_handler.h"

#include "led_strip.h" // LED制御用のライブラリ
#include "esp_timer.h" // 定期タイマー用のライブラリ
#include "main_PWM.h"  // 自作したPWM制御のヘッダー

#include "uart_to_camera.h" // 有線通信用
#include "pir_sensor.h"     // センサーおよびモード管理用
#include "buzzer_cmd.h"     // 格ゲーコマンド＆ブザー演奏機能用

#define LED_GPIO_PIN 38 // 通信状態確認LED
#define MAX_LEDS 1

static const char *TAG = "ESP_NOW_RX";

// --- 状態保持用変数 ---
static int last_rssi = -100; // 初期値
static int64_t last_packet_time = 0; // 最後にパケットが届いた時間

static led_strip_handle_t led_strip;
static esp_timer_handle_t led_update_timer;

static int success_count = 100;         // 通信スコア（初期値）
#define HISTORY_WEIGHT 100              // スコアの最大値
static bool data_received_flag = false; // データを受信したかどうかのフラグ

static uint8_t last_btn0_state = 0; // 非常停止トグル用（前回の状態）
static uint32_t last_toggle_time = 0; // 前回切り替えた時間

static uint8_t last_btn1_state = 0; // コマンド実行用（ボタン1の前回の状態）
static uint8_t last_btn3_state = 0; // モード切り替え用（ボタン3の前回の状態）

// 定期的に呼ばれるタイマー関数（20msごとに実行されて色を計算）
static void led_update_timer_callback(void* arg) {
    if (data_received_flag) {
        if (success_count < HISTORY_WEIGHT) success_count++;
    } else {
        if (success_count > 0) success_count--;
    }
    data_received_flag = false;

    float ratio = (float)success_count / HISTORY_WEIGHT;
    uint32_t red   = (uint32_t)(255 * (1.0 - ratio));
    uint32_t green = (uint32_t)(255 * ratio);
    uint32_t blue  = 0;

    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}

// LEDの初期設定を行う関数
void led_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_PIN,
        .max_leds = MAX_LEDS,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, 
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip); 
}

// ESP-NOWでデータを受信したときに実行されるコールバック関数
void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    controller_data_t received_data;
    
    if (len == sizeof(controller_data_t)) {
        memcpy(&received_data, data, sizeof(received_data));

        last_rssi = info->rx_ctrl->rssi;
        last_packet_time = esp_timer_get_time();

        data_received_flag = true;

        // 現在の動作モードを取得
        robot_mode_t current_mode = get_current_robot_mode();

        // 1. 格ゲーコマンド履歴バッファへの追記（いつでも裏で記録は行う）
        record_buzzer_command_input(received_data.dir);

        // 2. コントローラーから届いたレバー方向（1〜9）をPWM値に変換
        uint32_t rf = 0, rr = 0, lf = 0, lr = 0;
        
        // 通常走行モード（MODE_DRIVE）のときのみモーターを駆動させるガード
        if (current_mode == MODE_DRIVE) {
            switch (received_data.dir) {
                case 8: rf = 1023; rr = 0;    lf = 1023; lr = 0;    break; // 前進
                case 2: rf = 0;    rr = 1023; lf = 0;    lr = 1023; break; // 後進
                case 6: rf = 0;    rr = 1023; lf = 1023; lr = 0;    break; // 右超信地旋回
                case 4: rf = 1023; rr = 0;    lf = 0;    lr = 1023; break; // 左超信地旋回
                case 9: rf = 511;  rr = 0;    lf = 1023; lr = 0;    break; // 右斜め前
                case 7: rf = 1023; rr = 0;    lf = 511;  lr = 0;    break; // 左斜め前
                case 3: rf = 0;    rr = 511;  lf = 0;    lr = 1023; break; // 右斜め後ろ
                case 1: rf = 0;    rr = 1023; lf = 0;    lr = 511;  break; // 左斜め後ろ
                case 5: 
                default: rf = 0;  rr = 0;    lf = 0;    lr = 0;    break; // 停止
            }
        } else {
            // アタッチメントモード中は入力を無視して必ず停止を指示する
            rf = 0; rr = 0; lf = 0; lr = 0;
        }

        set_target_crawler_drive_4ch(rf, rr, lf, lr);

        // -------------------------------------------------------------
        // 停止機能:リモコンのボタン0入力によるトグル式停止モードの判定
        // -------------------------------------------------------------
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (current_time - last_toggle_time > 300) {
            if (received_data.btn[0] == 1 && last_btn0_state == 0) {
                toggle_emergency_stop_mode(); 
                last_toggle_time = current_time;
            }
        }
        last_btn0_state = received_data.btn[0];

        // -------------------------------------------------------------
        // モード切替機能: ボタン3 (btn[3]) の立ち上がりを検知して切り替え
        // -------------------------------------------------------------
        if (received_data.btn[3] == 1 && last_btn3_state == 0) {
            toggle_robot_mode(); // DRIVE ⇄ ATTACHMENT を切り替える
            current_mode = get_current_robot_mode(); // モード変数を更新
        }
        last_btn3_state = received_data.btn[3];

        // -------------------------------------------------------------
        // コマンド判定機能: ボタン1 (btn[1]) でコマンドトリガーを実行
        // -------------------------------------------------------------
        if (current_mode == MODE_ATTACHMENT) {
            // アタッチメントモード中、かつボタン1を押した瞬間
            if (received_data.btn[1] == 1 && last_btn1_state == 0) {
                //関数からコマンド成功・失敗の結果を受け取る
                bool is_success = check_and_trigger_buzzer_command(); 
                
                //判定結果によってカメラマイコンへ送る文字を分岐させる
                if (is_success) {
                    uart_send_string("CMD:SUCCESS\n");
                } else {
                    uart_send_string("CMD:FAILED\n");
                }
            }
        }
        last_btn1_state = received_data.btn[1];

        // 有線でカメラマイコン（XIAO）へ中継送信
        char uart_buf[128];
        snprintf(uart_buf, sizeof(uart_buf), "RSSI:%d,DIR:%d,BTN0:%d,MODE:%s\n", 
                 last_rssi, received_data.dir, received_data.btn[0], get_mode_name(current_mode));
        
        uart_send_string(uart_buf);

    } else {
        ESP_LOGW(TAG, "データサイズ不一致: 受信=%d, 想定=%d", len, sizeof(controller_data_t));
    }
}

// Wi-FiおよびESP-NOW機能の初期化を行う関数
void wifi_now_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));

    uart_to_camera_init();

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    led_init();

    const esp_timer_create_args_t timer_args = {
        .callback = &led_update_timer_callback,
        .name = "led_update_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &led_update_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(led_update_timer, 20000));

    ESP_LOGI(TAG, "受信待機中（LEDステータス監視有効）...");
}

// メイン側へデータを送信するための関数（現在未使用）
void send_data_to_main(controller_data_t *data) {
    // 未使用
}

// コントローラーとの接続状態（通信が途絶えていないか）を取得する関数
bool get_controller_connection_status(void) {
    int64_t current_time = esp_timer_get_time();
    return (current_time - last_packet_time) < 2000000;
}

// 直近の受信電波強度（RSSI）を取得する関数
int get_controller_rssi(void) {
    return last_rssi;
}
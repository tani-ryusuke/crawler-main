#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_now_handler.h"
#include "main_PWM.h"
#include "buzzer_cmd.h"
#include "pir_sensor.h"
#include "uart_to_camera.h"
#include "esp_timer.h"
#include "driver/ledc.h"

static const char *TAG = "MAIN_UNIT";

// --- バックグラウンド処理: ブザー制御用タスク変数 ---
static uint32_t buzzer_req_freq = 0;
static uint32_t buzzer_req_duration = 0;

/**
 * @brief ブザーのリクエストを監視し、非同期で再生処理を実行するタスク関数
 * @param pvParameters タスクに渡すパラメータ（未使用）
 */
void buzzer_task(void *pvParameters) {
    while (1) {
        // 単音の再生リクエストが存在するかを確認
        if (buzzer_req_freq > 0) {
            uint32_t f = buzzer_req_freq;
            uint32_t d = buzzer_req_duration;
            buzzer_req_freq = 0; // リクエストを初期化
            play_beep(f, d);
        }

        // メロディの再生リクエストを取得して実行
        int melody = get_requested_melody();
        if (melody > 0) {
            play_melody_internal(melody);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ミリ秒ごとに処理を周期実行
    }
}

/**
 * @brief ブザーの鳴動を別タスクへ非同期で要求する関数
 * @param freq 周波数 (Hz)
 * @param duration 継続時間 (ミリ秒)
 */
void request_beep(uint32_t freq, uint32_t duration) {
    buzzer_req_freq = freq;
    buzzer_req_duration = duration;
}

// --- センサーの監視とアラート処理 ---
/**
 * @brief PIRセンサーの状態を監視し、検知時に警告処理と外部通信を行う関数
 */
static void handle_pir_sensor_alert(void) {
    bool is_detected = is_pir_sensor_detected(); 
    
    // センサーの状態に変化があった場合のみログを出力
    static bool last_printed_state = false;
    if (is_detected != last_printed_state) {
        ESP_LOGI("DEBUG_PIR", "Filtered Sensor State Changed: %s", is_detected ? "DETECTED" : "IDLE");
        last_printed_state = is_detected;
    }

    // 通常走行モード時のみ警報チェックを実施
    if (get_current_robot_mode() == MODE_DRIVE) {
        static int64_t last_alert_time = 0; 
        int64_t current_time = esp_timer_get_time() / 1000;

        if (is_detected) { 
            if (current_time - last_alert_time > 1000) {
                // ブザー鳴動を非同期で要求
                request_beep(NOTE_C5, 200); 
                
                // カメラマイコンへセンサー検知を通知
                uart_send_string("ALERT:SENSOR_DETECTED\n");
                ESP_LOGW(TAG, "ALERT: PIR Sensor Detected - UART Sent");
                
                last_alert_time = current_time;
            }
        }
    }
}

/**
 * @brief メインアプリケーションのエントリーポイント
 */
void app_main(void)
{
    // 各ハードウェアおよび通信機能の初期化
    init_motor_pwm();
    init_buzzer_pwm();      

    init_pir_sensor();      
    wifi_now_init();

    // ブザー制御用のバックグラウンドタスクを生成
    xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Main Loop Started");
    
    while (1) {
        // 1. モーターの加減速制御を最優先で実行
        update_motor_soft_start();
        
        // 2. PIRセンサーの状態監視とアラート処理
        handle_pir_sensor_alert();
        
        // 3. メインループの周期を20ミリ秒に維持
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
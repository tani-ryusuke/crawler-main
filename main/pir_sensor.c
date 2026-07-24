#include "pir_sensor.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PIR_SENSOR";
static robot_mode_t current_mode = MODE_DRIVE;

// 信頼度メーター用の変数（関数をまたいでも値を保持）
static int confidence_meter = 0;
static bool is_currently_detected = false;

/**
 * @brief PIRセンサー用のGPIOピンを初期化する関数
 */
void init_pir_sensor(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_PIR_SENSOR),
        .mode         = GPIO_MODE_INPUT,
        // ※もし「1」が出続ける等の誤動作があれば PULLUP_DISABLE に変更
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO CONFIG FAILED! Error code: %d", err);
    } else {
        ESP_LOGI(TAG, "GPIO CONFIG SUCCESS");
    }
}

/**
 * @brief センサーピンの状態を読み取って返す (ヒステリシス処理付き)
 */
bool is_pir_sensor_detected(void) {
    // センサーのレベルを1回だけ取得 (ブロック処理を削除)
    int raw_val = gpio_get_level(GPIO_PIR_SENSOR);

    // 信頼度メーターの加減算
    if (raw_val == 1) { // 検知時
        if (confidence_meter < 10) confidence_meter++;
    } else { // 非検知時
        if (confidence_meter > 0) confidence_meter--;
    }

    // 5以上で検知ON、2以下で検知OFF (間に遊びを持たせてチラつきを防止)
    if (confidence_meter >= 5) {
        is_currently_detected = true;
    } else if (confidence_meter <= 2) {
        is_currently_detected = false;
    }
    
    return is_currently_detected;
}

/**
 * @brief 現在のモードを取得する関数
 */
robot_mode_t get_current_robot_mode(void) { 
    return current_mode; 
}

/**
 * @brief モードを直接設定する関数（安全のためMODE_MAX未満かチェック）
 */
void set_current_robot_mode(robot_mode_t mode) { 
    if (mode < MODE_MAX) current_mode = mode; 
}

/**
 * @brief ボタン1などでモードを交互に切り替える関数 (DRIVE ⇄ ATTACHMENT)
 */
void toggle_robot_mode(void) {
    ESP_LOGI(TAG, "★モード切替関数が呼び出されました！");
    current_mode = (current_mode + 1) % MODE_MAX;
    ESP_LOGI(TAG, "モード切替 ➔ %s", get_mode_name(current_mode));
}

/**
 * @brief モード名を文字列で返す関数 (ログ出力や通知用)
 */
const char* get_mode_name(robot_mode_t mode) {
    switch (mode) {
        case MODE_DRIVE:      return "DRIVE";
        case MODE_ATTACHMENT: return "ATTACHMENT";
        default:              return "UNKNOWN";
    }
}
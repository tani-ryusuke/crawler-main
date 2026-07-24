#include "buzzer_cmd.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BUZZER_CMD";

#define BUF_SIZE 20
static int dirBuf[BUF_SIZE] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
static int bufIndex = 0;

// 非同期メロディリクエスト用の変数
static volatile int requested_melody = 0;

// ブザー用PWMの初期化処理
void init_buzzer_pwm(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_11_BIT,
        .freq_hz          = 2000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = GPIO_BUZZER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "ブザーPWMの初期化完了");
}

// 指定した周波数と時間でビープ音を鳴らす関数
void play_beep(uint32_t freq_hz, uint32_t duration_ms) {
    if (freq_hz == 0) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_50PCT);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// メロディの内部再生処理
void play_melody_internal(int cmd_type) {
    if (cmd_type == 1) { // 波動拳：テトリス
        play_beep(NOTE_E5, 320); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_B4, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_C5, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_D5, 320); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_C5, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_B4, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_A4, 350); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_A4, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_C5, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_E5, 350); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_D5, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_C5, 150); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_B4, 320); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_B4, 160); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_C5, 160); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_D5, 320); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_E5, 350); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_C5, 280); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_A4, 400); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_A4, 550);
    }
    else if (cmd_type == 2) { // 昇竜拳：ダース・ベイダーのテーマ
        play_beep(NOTE_G4, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_G4, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_G4, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_DS4, 350); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_AS4, 150); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G4, 500);  vTaskDelay(pdMS_TO_TICKS(80));
        play_beep(NOTE_DS4, 350); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_AS4, 150); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G4, 900);  vTaskDelay(pdMS_TO_TICKS(300));

        play_beep(NOTE_D5, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_D5, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_D5, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_DS5, 350); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_AS4, 150); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_FS4, 500); vTaskDelay(pdMS_TO_TICKS(80));
        play_beep(NOTE_DS4, 350); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_AS4, 150); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G4, 1200); vTaskDelay(pdMS_TO_TICKS(300));

        play_beep(NOTE_G5, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_G4, 250);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G4, 250);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G5, 500);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_FS5, 250); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_F5, 250);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_E5, 250);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_DS5, 250); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_E5, 700);  vTaskDelay(pdMS_TO_TICKS(100));

        play_beep(NOTE_GS4, 350); vTaskDelay(pdMS_TO_TICKS(50));
        play_beep(NOTE_CS5, 500); vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_C5, 350);  vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_B4, 350);  vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_AS4, 200); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_A4, 200);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_AS4, 400); vTaskDelay(pdMS_TO_TICKS(100));
        play_beep(NOTE_DS4, 300); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_FS4, 500); vTaskDelay(pdMS_TO_TICKS(200));

        play_beep(NOTE_DS4, 350); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_AS4, 150); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G4, 500);  vTaskDelay(pdMS_TO_TICKS(100));
        play_beep(NOTE_DS4, 350); vTaskDelay(pdMS_TO_TICKS(40));
        play_beep(NOTE_AS4, 150); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G4, 1500);
    }
    else if (cmd_type == 3) { // 竜巻旋風脚：パックマン
        play_beep(NOTE_B4, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_B5, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_FS5, 110); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_DS5, 110); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_B5, 220);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_FS5, 240); vTaskDelay(pdMS_TO_TICKS(90));

        play_beep(NOTE_C5, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_C6, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_G5, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_E5, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_C6, 220);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_G5, 240);  vTaskDelay(pdMS_TO_TICKS(90));

        play_beep(NOTE_B4, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_B5, 110);  vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_FS5, 110); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_DS5, 110); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_B5, 220);  vTaskDelay(pdMS_TO_TICKS(70));
        play_beep(NOTE_FS5, 240); vTaskDelay(pdMS_TO_TICKS(90));

        play_beep(NOTE_DS5, 130); vTaskDelay(pdMS_TO_TICKS(22));
        play_beep(NOTE_E5,  80);  vTaskDelay(pdMS_TO_TICKS(22));
        play_beep(NOTE_F5,  80);  vTaskDelay(pdMS_TO_TICKS(22));
        play_beep(NOTE_F5,  100); vTaskDelay(pdMS_TO_TICKS(25));
        play_beep(NOTE_FS5, 80);  vTaskDelay(pdMS_TO_TICKS(22));
        play_beep(NOTE_G5,  80);  vTaskDelay(pdMS_TO_TICKS(22));
        play_beep(NOTE_G5,  110); vTaskDelay(pdMS_TO_TICKS(30));
        play_beep(NOTE_GS5, 70);  vTaskDelay(pdMS_TO_TICKS(18));
        play_beep(NOTE_A5,  70);  vTaskDelay(pdMS_TO_TICKS(18));
        play_beep(NOTE_B5,  200);
    }
}

// メロディの再生をリクエストする関数
void request_melody(int cmd_type) {
    requested_melody = cmd_type;
}

// リクエストされたメロディを取得し、変数をリセットする関数
int get_requested_melody(void) {
    int m = requested_melody;
    requested_melody = 0;
    return m;
}

// 方向入力の履歴バッファに新しい入力を追加する内部関数
void push_direction_history(int dir) {
    dirBuf[bufIndex] = dir;
    bufIndex = (bufIndex + 1) % BUF_SIZE;
}

// esp_now_handler.c から呼び出すための方向入力記録ラッパー関数
void record_buzzer_command_input(int dir) {
    push_direction_history(dir);
}

// 波動拳コマンド（236）の成立を判定する関数
bool check_hadou(void) {
    bool got2 = false, got3 = false;
    for (int i = 0; i < BUF_SIZE; i++) {
        int idx = (bufIndex + i) % BUF_SIZE;
        int d = dirBuf[idx];
        if (!got2 && d == 2) { got2 = true; }
        else if (got2 && !got3 && d == 3) { got3 = true; }
        else if (got3 && d == 6) { return true; }
    }
    return false;
}

// 昇竜拳コマンド（623）の成立を判定する関数
bool check_shoryu(void) {
    bool got6 = false, got2 = false;

    for (int i = 0; i < BUF_SIZE; i++) {
        int idx = (bufIndex + i) % BUF_SIZE;
        int d = dirBuf[idx];

        if (!got6 && d == 6) {
            got6 = true;
        }
        else if (got6 && !got2 && d == 2) {
            got2 = true;
        }
        else if (got2 && d == 3) {
            return true;
        }
    }
    return false;
}

// 竜巻旋風脚コマンド（214）の成立を判定する関数
bool check_tatsumaki(void) {
    bool got2 = false, got1 = false;
    for (int i = 0; i < BUF_SIZE; i++) {
        int idx = (bufIndex + i) % BUF_SIZE;
        int d = dirBuf[idx];
        if (!got2 && d == 2) { got2 = true; }
        else if (got2 && !got1 && d == 1) { got1 = true; }
        else if (got1 && d == 4) { return true; }
    }
    return false;
}

// 各コマンドの成立をチェックし、対応する処理やメロディのリクエストを行う関数
bool check_and_trigger_buzzer_command(void) {
    if (check_shoryu()) {
        ESP_LOGI(TAG, "コマンド成功：昇竜拳！");
        request_melody(2);
        return true;
    } 
    else if (check_hadou()) {
        ESP_LOGI(TAG, "コマンド成功：波動拳！");
        request_melody(1);
        return true;
    } 
    else if (check_tatsumaki()) {
        ESP_LOGI(TAG, "コマンド成功：竜巻旋風脚！");
        request_melody(3);
        return true;
    } 
    else {
        ESP_LOGI(TAG, "コマンド不一致");
        return false;
    }
}
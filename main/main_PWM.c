#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "main_PWM.h"
#include "esp_now_handler.h"

// モーターを接続するGPIOピンの番号を設定しています
#define MOTOR_RIGHT_FWD_IO      (1)   // 右モーターの正転用ピン
#define MOTOR_RIGHT_REV_IO      (2)   // 右モーターの逆転用ピン
#define MOTOR_LEFT_FWD_IO       (17)  // 左モーターの正転用ピン
#define MOTOR_LEFT_REV_IO       (18)  // 左モーターの逆転用ピン

// PWM制御の解像度を10ビットに設定（出力強度を0から1023の範囲で指定）
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT 
// PWM信号の周波数を200ヘルツに設定
#define LEDC_FREQUENCY          (200)            

// 4つのモーター出力に対して、それぞれ独立したPWMチャンネルを割り当て
#define CH_RIGHT_FWD            LEDC_CHANNEL_0
#define CH_RIGHT_REV            LEDC_CHANNEL_1
#define CH_LEFT_FWD             LEDC_CHANNEL_2
#define CH_LEFT_REV             LEDC_CHANNEL_3

// モーターの現在の出力値を記憶する変数（rfは右正転、rrは右逆転、lfは左正転、lrは左逆転）
static uint32_t current_rf = 0, current_rr = 0, current_lf = 0, current_lr = 0;
// 通信などで指示されたモーターの目標出力値を記憶する変数
static uint32_t target_rf = 0,  target_rr = 0,  target_lf = 0,  target_lr = 0;

// 緊急停止の状態を管理するフラグ（trueで停止、falseで通常走行）
static volatile bool g_stop_mode = false;


// モーターを制御するためのPWM機能を初期化する関数
void init_motor_pwm(void)
{
    // 4つのチャンネルで共通して使用するタイマーの設定
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    // 設定をタイマーに反映させ、エラーがないかチェック
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 各モーターピンとPWMチャンネルを紐付けるための設定
    ledc_channel_config_t ledc_ch[4] = {
        { .gpio_num = MOTOR_RIGHT_FWD_IO, .channel = CH_RIGHT_FWD, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_RIGHT_REV_IO, .channel = CH_RIGHT_REV, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_LEFT_FWD_IO,  .channel = CH_LEFT_FWD,  .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_LEFT_REV_IO,  .channel = CH_LEFT_REV,  .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 }
    };

    // 4つのチャンネルすべてに設定を適用
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch[i]));
    }

    printf("4-Channel Motor PWM Initialized (200Hz, 10bit)\n");
}

// 4つのPWM出力を実際に同時に更新してモーターを動かす関数
void set_crawler_drive_4ch(uint32_t r_fwd, uint32_t r_rev, uint32_t l_fwd, uint32_t l_rev)
{
    // 右モーターの正転出力を設定（1023を超える値は1023に制限）
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_FWD, r_fwd > 1023 ? 1023 : r_fwd));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_FWD));

    // 右モーターの逆転出力を設定
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_REV, r_rev > 1023 ? 1023 : r_rev));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_REV));

    // 左モーターの正転出力を設定
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_FWD,  l_fwd > 1023 ? 1023 : l_fwd));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_FWD));

    // 左モーターの逆転出力を設定
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_REV,  l_rev > 1023 ? 1023 : l_rev));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_REV));
}

// 無線通信などから新しい移動の目標値を受け取り、変数に保存する関数
void set_target_crawler_drive_4ch(uint32_t rf, uint32_t rr, uint32_t lf, uint32_t lr)
{
    target_rf = rf;
    target_rr = rr;
    target_lf = lf;
    target_lr = lr;
}

// 現在の出力値を、指定されたステップ量（変化量）ずつ目標値に近づけるための計算関数
static uint32_t approach_target(uint32_t current, uint32_t target, uint32_t step) {
    if (current < target) {
        // 現在値が目標値より小さい場合は数値を増す
        return (current + step > target) ? target : current + step;
    } else if (current > target) {
        // 現在値が目標値より大きい場合は数値を減らす（アンダーフローを防ぐ処理）
        return (current > step) ? (current - step < target ? target : current - step) : 0;
    }
    return current;
}

// モーターの急発進や急停止を抑え、安全対策を施しながら出力を滑らかに更新するメイン関数
void update_motor_soft_start(void)
{
    // 通常時の加減速のスピードを決める基準値
    uint32_t base_step = 150; 

    // データのエラーなどで正転と逆転の指示が同時に出た場合、安全のために両方ゼロ（停止）
    if (target_rf > 0 && target_rr > 0) { target_rf = 0; target_rr = 0; }
    if (target_lf > 0 && target_lr > 0) { target_lf = 0; target_lr = 0; }

    // 各モーターの最終的な目標値を格納する変数
    uint32_t eff_target_rf = target_rf;
    uint32_t eff_target_rr = target_rr;
    uint32_t eff_target_lf = target_lf;
    uint32_t eff_target_lr = target_lr;

    // 緊急停止モードが有効になっているかを確認
    if (g_stop_mode) {
        // 全モーターの目標出力を強制的にゼロに
        eff_target_rf = 0;
        eff_target_rr = 0;
        eff_target_lf = 0;
        eff_target_lr = 0;
        
        // 停止時は通常時よりも素早く出力を落とすために変化量を大きく
        base_step = 500; 
    }

    // 左右のキャタピラを逆方向に回すその場旋回（超信地旋回）の指示が出ているかを判定
    bool is_turning_right = (target_rr > 0 && target_lf > 0 && target_rf == 0 && target_lr == 0);
    bool is_turning_left  = (target_rf > 0 && target_lr > 0 && target_rr == 0 && target_lf == 0);

    // その場旋回のときは、動きが激しくなりすぎないよう出力を一律で50パーセントに制限
    if (is_turning_right || is_turning_left) {
        const uint32_t turn_limit_percent = 50; 

        eff_target_rf = (eff_target_rf * turn_limit_percent) / 100;
        eff_target_rr = (eff_target_rr * turn_limit_percent) / 100;
        eff_target_lf = (eff_target_lf * turn_limit_percent) / 100;
        eff_target_lr = (eff_target_lr * turn_limit_percent) / 100;
    }

    // 逆電流やショートを防ぐため、逆方向の出力がまだ残っている場合は新しい加速指示をゼロにして待機
    if (current_rr > 0) eff_target_rf = 0; // 右逆転が出力中なら、右正転の目標を0に
    if (current_rf > 0) eff_target_rr = 0; // 右正転が出力中なら、右逆転の目標を0に

    if (current_lr > 0) eff_target_lf = 0; // 左逆転が出力中なら、左正転の目標を0に
    if (current_lf > 0) eff_target_lr = 0; // 左正転が出力中なら、左逆転の目標を0に

    // 目標値と現在値の大きい方を基準にして、出力変化の比率を計算する準備
    uint32_t ref_rf = (eff_target_rf > current_rf) ? eff_target_rf : current_rf;
    uint32_t ref_rr = (eff_target_rr > current_rr) ? eff_target_rr : current_rr;
    uint32_t ref_lf = (eff_target_lf > current_lf) ? eff_target_lf : current_lf;
    uint32_t ref_lr = (eff_target_lr > current_lr) ? eff_target_lr : current_lr;

    // 各モーターの現在の出力の大きさに比例した、適切なステップ量を計算
    uint32_t step_rf = (ref_rf * base_step) / 1023;
    uint32_t step_rr = (ref_rr * base_step) / 1023;
    uint32_t step_lf = (ref_lf * base_step) / 1023;
    uint32_t step_lr = (ref_lr * base_step) / 1023;

    // 計算結果がゼロになってしまい、値が変化しなくなるフリーズ現象を防ぐための処理
    if (ref_rf > 0 && step_rf == 0) step_rf = 1;
    if (ref_rr > 0 && step_rr == 0) step_rr = 1;
    if (ref_lf > 0 && step_lf == 0) step_lf = 1;
    if (ref_lr > 0 && step_lr == 0) step_lr = 1;

    // 計算したステップ数を使って、現在値を目標値に向けて少しずつ変化
    current_rf = approach_target(current_rf, eff_target_rf, step_rf);
    current_rr = approach_target(current_rr, eff_target_rr, step_rr);
    current_lf = approach_target(current_lf, eff_target_lf, step_lf);
    current_lr = approach_target(current_lr, eff_target_lr, step_lr);

    // 最終的に計算された現在値を、実際のPWMコントローラーへ出力してモーターを駆動
    set_crawler_drive_4ch(current_rf, current_rr, current_lf, current_lr);
}

// 緊急停止モードの有効と無効を、呼び出されるたびに反転させる関数
void toggle_emergency_stop_mode(void)
{
    // フラグの状態を反転させています（trueならfalseに、falseならtrueにします）
    g_stop_mode = !g_stop_mode;
    
    if (g_stop_mode) {
        printf("[SYSTEM] Emergency Stop Mode: ENABLED (Motors Locked)\n");
    } else {
        printf("[SYSTEM] Emergency Stop Mode: DISABLED (Ready to Run)\n");
    }
}
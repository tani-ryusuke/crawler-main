#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "main_PWM.h"
#include "esp_now_handler.h"

// ピン配置の設定
#define MOTOR_RIGHT_FWD_IO      (1)   // 右正転 (GPIO1)
#define MOTOR_RIGHT_REV_IO      (2)   // 右逆転 (GPIO2)
#define MOTOR_LEFT_FWD_IO       (17)  // 左正転 (GPIO17)
#define MOTOR_LEFT_REV_IO       (18)  // 左逆転 (GPIO18)

#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // 分解能 10bit (0 〜 1023)
#define LEDC_FREQUENCY          (1000)            // 周波数

// 4つのチャンネルをそれぞれ割り当て
#define CH_RIGHT_FWD            LEDC_CHANNEL_0
#define CH_RIGHT_REV            LEDC_CHANNEL_1
#define CH_LEFT_FWD             LEDC_CHANNEL_2
#define CH_LEFT_REV             LEDC_CHANNEL_3

// 現在値と、目標値を保持する静的変数
static uint32_t current_rf = 0, current_rr = 0, current_lf = 0, current_lr = 0;
static uint32_t target_rf = 0,  target_rr = 0,  target_lf = 0,  target_lr = 0;

// トグル式の停止モード（true=停止、false=走行）を管理するフラグ
static volatile bool g_stop_mode = false;


/**
 * @brief 4チャンネル分のモーターPWM（LEDC）を初期化する関数
 */
void init_motor_pwm(void)
{
    // タイマーの設定（4chすべて共通のタイマー0を使用）
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 各チャンネルの設定用構造体配列
    ledc_channel_config_t ledc_ch[4] = {
        { .gpio_num = MOTOR_RIGHT_FWD_IO, .channel = CH_RIGHT_FWD, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_RIGHT_REV_IO, .channel = CH_RIGHT_REV, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_LEFT_FWD_IO,  .channel = CH_LEFT_FWD,  .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_LEFT_REV_IO,  .channel = CH_LEFT_REV,  .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 }
    };

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch[i]));
    }

    printf("4-Channel Motor PWM Initialized (200Hz, 10bit)\n");
}

/**
 * @brief 4つのPWM出力を一括で更新する関数
 */
void set_crawler_drive_4ch(uint32_t r_fwd, uint32_t r_rev, uint32_t l_fwd, uint32_t l_rev)
{
    // 右正転
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_FWD, r_fwd > 1023 ? 1023 : r_fwd));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_FWD));
    // 右逆転
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_REV, r_rev > 1023 ? 1023 : r_rev));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT_REV));
    // 左正転
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_FWD,  l_fwd > 1023 ? 1023 : l_fwd));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_FWD));
    // 左逆転
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_REV,  l_rev > 1023 ? 1023 : l_rev));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_LEFT_REV));
}

/**
 * @brief ESP-NOW受信時に「目標値」だけをセットする関数
 */
void set_target_crawler_drive_4ch(uint32_t rf, uint32_t rr, uint32_t lf, uint32_t lr)
{
    target_rf = rf;
    target_rr = rr;
    target_lf = lf;
    target_lr = lr;
}

/**
 * @brief 現在値を目標値に近づけるための計算用補助関数
 */
static uint32_t approach_target(uint32_t current, uint32_t target, uint32_t step) {
    if (current < target) {
        return (current + step > target) ? target : current + step;
    } else if (current > target) {
        return (current > step) ? (current - step < target ? target : current - step) : 0;
    }
    return current;
}

/**
 * @brief 逆電流防止・旋回出力制限付き：加減速を管理するメイン関数
 */
void update_motor_soft_start(void)
{
    // 加減速ステップ数：全方向共通で 150（約0.14秒で最大）
    uint32_t base_step = 150; 

    // 万が一、通信バグ等で正転と逆転の目標値が同時に届いたら、安全のため両方0（停止）にする
    if (target_rf > 0 && target_rr > 0) { target_rf = 0; target_rr = 0; }
    if (target_lf > 0 && target_lr > 0) { target_lf = 0; target_lr = 0; }

    // 実質的な目標値（eff_target）を作成
    uint32_t eff_target_rf = target_rf;
    uint32_t eff_target_rr = target_rr;
    uint32_t eff_target_lf = target_lf;
    uint32_t eff_target_lr = target_lr;

    // トグル式の停止モードの評価
    if (g_stop_mode) {
        // 停止モード中なら、全モーターの目標値を強制的に「0」にする
        eff_target_rf = 0;
        eff_target_rr = 0;
        eff_target_lf = 0;
        eff_target_lr = 0;
        
        // 停止モードに入るときは約0.04秒で電力をカット
        base_step = 500; 
    }

    // ★左右のその場旋回（右超信地旋回、または左超信地旋回）の命令が出ているかチェック
    // 右旋回条件：右モーターが逆転(rr) かつ 左モーターが正転(lf) ※他は0
    // 左旋回条件：右モーターが正転(rf) かつ 左モーターが逆転(lr) ※他は0
    bool is_turning_right = (target_rr > 0 && target_lf > 0 && target_rf == 0 && target_lr == 0);
    bool is_turning_left  = (target_rf > 0 && target_lr > 0 && target_rr == 0 && target_lf == 0);

    // 左右旋回操作のときだけ、最高回転数（パワー）を一律で50%に
    if (is_turning_right || is_turning_left) {
        const uint32_t turn_limit_percent = 50; 

        eff_target_rf = (eff_target_rf * turn_limit_percent) / 100;
        eff_target_rr = (eff_target_rr * turn_limit_percent) / 100;
        eff_target_lf = (eff_target_lf * turn_limit_percent) / 100;
        eff_target_lr = (eff_target_lr * turn_limit_percent) / 100;
    }

    // 安全対策：正転と逆転が同時にONになるのを防ぐ
    // 相手方のモーター出力がまだ残っている間は、自分の目標値を一時的に「0」にして加速を待機させる
    if (current_rr > 0) eff_target_rf = 0; // 逆転が出力中なら、正転の目標を0にする
    if (current_rf > 0) eff_target_rr = 0; // 正転が出力中なら、逆転の目標を0にする

    if (current_lr > 0) eff_target_lf = 0; // 逆転が出力中なら、正転の目標を0にする
    if (current_lf > 0) eff_target_lr = 0; // 正転が出力中なら、逆転の目標を0にする

    // target(安全対策・制限後)とcurrentの「大きい方」を基準（ref）にする
    uint32_t ref_rf = (eff_target_rf > current_rf) ? eff_target_rf : current_rf;
    uint32_t ref_rr = (eff_target_rr > current_rr) ? eff_target_rr : current_rr;
    uint32_t ref_lf = (eff_target_lf > current_lf) ? eff_target_lf : current_lf;
    uint32_t ref_lr = (eff_target_lr > current_lr) ? eff_target_lr : current_lr;

    // それぞれのモーターの比率に合ったステップ量を計算
    uint32_t step_rf = (ref_rf * base_step) / 1023;
    uint32_t step_rr = (ref_rr * base_step) / 1023;
    uint32_t step_lf = (ref_lf * base_step) / 1023;
    uint32_t step_lr = (ref_lr * base_step) / 1023;

    // 計算結果が0になってフリーズするのを防ぐ
    if (ref_rf > 0 && step_rf == 0) step_rf = 1;
    if (ref_rr > 0 && step_rr == 0) step_rr = 1;
    if (ref_lf > 0 && step_lf == 0) step_lf = 1;
    if (ref_lr > 0 && step_lr == 0) step_lr = 1;

    // 計算したステップ数を使って、現在値を目標値に近づける
    current_rf = approach_target(current_rf, eff_target_rf, step_rf);
    current_rr = approach_target(current_rr, eff_target_rr, step_rr);
    current_lf = approach_target(current_lf, eff_target_lf, step_lf);
    current_lr = approach_target(current_lr, eff_target_lr, step_lr);

    // PWMコントローラーへ出力
    set_crawler_drive_4ch(current_rf, current_rr, current_lf, current_lr);
}

/**
 * @brief 通信イベントから安全に停止モードをトグル（反転）させる関数
 */
void toggle_emergency_stop_mode(void)
{
    g_stop_mode = !g_stop_mode;
    
    if (g_stop_mode) {
        printf("【SYSTEM】Emergency Stop Mode: ENABLED (Motors Locked)\n");
    } else {
        printf("【SYSTEM】Emergency Stop Mode: DISABLED (Ready to Run)\n");
    }
}
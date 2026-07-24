#ifndef MAIN_PWM_H
#define MAIN_PWM_H

#include <stdint.h>

// モーターPWMの初期化を行う関数の宣言
void init_motor_pwm(void);

// 4チャンネルのPWM出力を一括で更新する関数の宣言
void set_crawler_drive_4ch(uint32_t r_fwd, uint32_t r_rev, uint32_t l_fwd, uint32_t l_rev);

// ソフトスタート用の目標値をセットする関数の宣言
void set_target_crawler_drive_4ch(uint32_t rf, uint32_t rr, uint32_t lf, uint32_t lr);

// モーターの加減速を管理し、現在値を更新するメイン関数の宣言
void update_motor_soft_start(void);

// 非常停止モードの有効/無効を切り替える関数の宣言
void toggle_emergency_stop_mode(void);

#endif // MAIN_PWM_H
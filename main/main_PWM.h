#ifndef MAIN_PWM_H
#define MAIN_PWM_H

#include <stdint.h>

// 関数宣言
void init_motor_pwm(void);
void set_crawler_drive_4ch(uint32_t r_fwd, uint32_t r_rev, uint32_t l_fwd, uint32_t l_rev);

// ソフトスタート用の関数宣言
void set_target_crawler_drive_4ch(uint32_t rf, uint32_t rr, uint32_t lf, uint32_t lr);
void update_motor_soft_start(void);

//停止モード切替用関数宣言
void toggle_emergency_stop_mode(void);

#endif // MAIN_PWM_H
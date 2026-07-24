#ifndef BUZZER_CMD_H
#define BUZZER_CMD_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"

// パッシブブザー用のGPIOピン定義
#define GPIO_BUZZER      GPIO_NUM_15   

// LEDC（PWM）の設定値定義
#define LEDC_TIMER       LEDC_TIMER_1
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL     LEDC_CHANNEL_4
#define LEDC_DUTY_50PCT  1024

// 音符の定義（オクターブ4 & 半音）
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494

// 音符の定義（オクターブ5 & 半音）
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988

// 高音（オクターブ6）
#define NOTE_C6  1047
#define NOTE_D6  1175
#define NOTE_E6  1319
#define NOTE_F6  1397

// 関数の宣言
void init_buzzer_pwm(void);
void play_beep(uint32_t freq_hz, uint32_t duration_ms);
void play_melody_internal(int cmd_type);

// 非同期メロディ制御用
void request_melody(int cmd_type);
int get_requested_melody(void);

// 履歴管理・コマンド判定関数
void push_direction_history(int dir);
bool check_hadou(void);
bool check_shoryu(void);
bool check_tatsumaki(void);

// 外部公開用関数
void record_buzzer_command_input(int dir);
bool check_and_trigger_buzzer_command(void);

#endif // BUZZER_CMD_H
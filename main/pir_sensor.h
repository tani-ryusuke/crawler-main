#ifndef PIR_SENSOR_H
#define PIR_SENSOR_H

#include <stdbool.h>
#include "driver/gpio.h"

#define GPIO_PIR_SENSOR   GPIO_NUM_21   // 赤外線センサー用のピン (GPIO 21)

// モードを「走行（センサー同時監視）」と「アタッチメント」の2つに集約
typedef enum {
    MODE_DRIVE = 0,     // 通常走行モード（バックグラウンドで赤外線センサーも同時に動作します）
    MODE_ATTACHMENT,    // アタッチメントモード（格ゲーコマンド有効）
    MODE_MAX
} robot_mode_t;

// PIRセンサーの初期化を行う関数の宣言
void init_pir_sensor(void);

// PIRセンサーが物体を検知しているかを取得する関数の宣言
bool is_pir_sensor_detected(void);

// 現在のロボットモードを取得する関数の宣言
robot_mode_t get_current_robot_mode(void);

// ロボットモードを直接設定する関数の宣言
void set_current_robot_mode(robot_mode_t mode);

// モードを交互に切り替える関数の宣言
void toggle_robot_mode(void);

// ロボットモードの名称を文字列で取得する関数の宣言
const char* get_mode_name(robot_mode_t mode);

#endif // PIR_SENSOR_H
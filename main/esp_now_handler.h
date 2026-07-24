#ifndef ESP_NOW_HANDLER_H
#define ESP_NOW_HANDLER_H

#include <stdbool.h>
#include "esp_now.h"

// 送信したいデータの構造体定義（コントローラー側と完全に一致させる）
typedef struct {
    int dir;          // テンキー方向 (1~9)
    bool btn[8];      // ボタン8個の状態 (true:ON / false:OFF)
} controller_data_t;

// Wi-FiおよびESP-NOWの初期化関数の宣言
void wifi_now_init(void);

// 送信関数の宣言（受信機側では中身は空ですが、定義は合わせておく）
void send_data_to_main(controller_data_t *data);

// コントローラーとの接続状態を取得する関数の宣言
bool get_controller_connection_status(void);

// コントローラーの電波強度（RSSI）を取得する関数の宣言
int get_controller_rssi(void);

#endif // ESP_NOW_HANDLER_H
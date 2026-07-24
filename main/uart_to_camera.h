#ifndef UART_TO_CAMERA_H
#define UART_TO_CAMERA_H

#include "driver/uart.h"

#define CAMERA_UART_NUM      UART_NUM_1
#define CAMERA_UART_TX_PIN   (4)  // 本体側のTXピン（XIAOのD7へ）
#define CAMERA_UART_RX_PIN   (5)  // 本体側のRXピン（XIAOのD6へ）

// UARTの初期化関数
void uart_to_camera_init(void);

// カメラマイコンへ文字列を送る関数
void uart_send_string(const char* str);

#endif // UART_TO_CAMERA_H
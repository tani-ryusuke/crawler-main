#include "uart_to_camera.h"
#include <string.h>

/**
 * @brief カメラマイコン（XIAO）とのUART通信を初期化する関数
 */
void uart_to_camera_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // 送信メインなのでバッファは最小限(1024)
    uart_driver_install(CAMERA_UART_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(CAMERA_UART_NUM, &uart_config);
    uart_set_pin(CAMERA_UART_NUM, CAMERA_UART_TX_PIN, CAMERA_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief カメラマイコンへ文字列データを送信する関数
 */
void uart_send_string(const char* str) {
    uart_write_bytes(CAMERA_UART_NUM, str, strlen(str));
}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now_handler.h"
//#include "cameraWi-Fi_manager.h"
#include "main_PWM.h"

void app_main(void)
{
    // 先にモーターの準備（PWM初期化）を終わらせる
    init_motor_pwm();
    
    // ESP-NOWの初期化（受信モード）
    wifi_now_init();

    //init_cameraWi_Fi_manager();     // Wi-Fi接続とサーバー起動（未実装）

    while (1) {
        //定期的にモーターの出力を少しずつ目標値に近づけるソフトスタート
        update_motor_soft_start();
        
        //1000ms(1秒)の待機から、20msの待機に変更
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
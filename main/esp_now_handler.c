#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_now_handler.h"

#include "led_strip.h" // LEDを制御するためのライブラリ
#include "esp_timer.h" // 定期的なタイマー処理を行うためのライブラリ
#include "main_PWM.h"  // 別ファイルで定義されているPWM制御の関数を使用するために読み込む

#define LED_GPIO_PIN 38 // 通信状態を確認するためのLEDが接続されているピン番号
#define MAX_LEDS 1 // 制御するLEDの個数

static const char *TAG = "ESP_NOW_RX"; // ログ出力時に使用する識別名

// 状態を保持するための変数群
static int last_rssi = -100; // 最後に受信したときの電波強度を記憶する変数。初期値はマイナス100
static int64_t last_packet_time = 0; // 最後にパケットが届いたシステム時間を記憶する変数

static led_strip_handle_t led_strip; // LEDを操作するための識別情報
static esp_timer_handle_t led_update_timer; // 定期タイマーを管理するための識別情報

static int success_count = 100;         // 通信の成功状態を表すスコア。初期値は100
#define HISTORY_WEIGHT 100              // 通信スコアの最大値
static bool data_received_flag = false; // 指定時間内にデータを受信したかどうかを判別するフラグ

static uint8_t last_btn0_state = 0; // 前回の処理時におけるボタンゼロの状態を記憶する変数
static uint32_t last_toggle_time = 0; // 前回停止モードを切り替えたシステム時間を記憶する変数

// 20ミリ秒ごとに自動で呼び出されLEDの色を計算して更新する関数
static void led_update_timer_callback(void* arg) {
    // この20ミリ秒の間にデータが届いていた場合はスコアを増やし届いていなければ減らす
    if (data_received_flag) {
        if (success_count < HISTORY_WEIGHT) success_count++;
    } else {
        if (success_count > 0) success_count--;
    }

    // 次の周期の判定のために受信フラグを偽に戻す
    data_received_flag = false;

    // 通信成功率を0.0から1.0の範囲で算出して色を滑らかに合成する
    float ratio = (float)success_count / HISTORY_WEIGHT;
    uint32_t red   = (uint32_t)(255 * (1.0 - ratio)); // 成功率が低いほど赤色が強くなる
    uint32_t green = (uint32_t)(255 * ratio);       // 成功率が高いほど緑色が強くなる
    uint32_t blue  = 0;

    // 計算した色をLEDに反映させる
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}

// LEDの初期設定を行う関数
void led_init(void) {
    // LEDの接続ピンや種類などを設定する
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO_PIN,
        .max_leds = MAX_LEDS,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, 
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    // 通信制御用の周辺機器の設定を行う
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };
    // 設定を反映させてLEDデバイスを作成し初期化として一度消灯させる
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip); 
}

// 無線データを受信したときに自動的に実行される関数
void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    controller_data_t received_data;
    
    // 受信したバイナリデータのサイズが想定通りであれば構造体に中身をコピーする
    if (len == sizeof(controller_data_t)) {
        memcpy(&received_data, data, sizeof(received_data));

        last_rssi = info->rx_ctrl->rssi; // 電波強度を保存する
        last_packet_time = esp_timer_get_time(); // 現在のシステム時間をマイクロ秒単位で取得して保存する

        data_received_flag = true; // データが正常に届いたことを示すフラグを真にする

        uint32_t rf = 0, rr = 0, lf = 0, lr = 0; // 4チャンネル分のモーター出力を格納する変数を初期化する

        // コントローラーから届いたレバーの方向を示す数値に応じて各モーターの出力を決定する
        switch (received_data.dir) {
            case 8: rf = 1023; rr = 0;    lf = 1023; lr = 0;    break; // 前進の処理
            case 2: rf = 0;    rr = 1023; lf = 0;    lr = 1023; break; // 後進の処理
            case 6: rf = 0;    rr = 1023; lf = 1023; lr = 0;    break; // 右超信地旋回の処理
            case 4: rf = 1023; rr = 0;    lf = 0;    lr = 1023; break; // 左超信地旋回の処理
            case 9: rf = 511;  rr = 0;    lf = 1023; lr = 0;    break; // 右斜め前進の処理
            case 7: rf = 1023; rr = 0;    lf = 511;  lr = 0;    break; // 左斜め前進の処理
            case 3: rf = 0;    rr = 511;  lf = 0;    lr = 1023; break; // 右斜め後進の処理
            case 1: rf = 0;    rr = 1023; lf = 0;    lr = 511;  break; // 左斜め後進の処理
            case 5: 
            default: rf = 0;   rr = 0;    lf = 0;    lr = 0;    break; // 停止の処理
        }

        // リモコンのボタンゼロが押されたときに停止モードを切り替えるトグル処理
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS; // 現在のシステム時間をミリ秒単位で取得する

        // ボタンの押しっぱなしによる連続動作を防ぐため前回の切り替えから300ミリ秒以内は処理を無視する
        if (current_time - last_toggle_time > 300) {
            
            // 前回はボタンが離されていて今回は押されている瞬間であるかを判定する
            if (received_data.btn[0] == 1 && last_btn0_state == 0) {
                
                toggle_emergency_stop_mode(); // モーター制御ファイル側の緊急停止切り替え関数を呼び出す
                
                last_toggle_time = current_time; // 切り替えた時間を更新してここから300ミリ秒のロックを開始する
            }
            
            last_btn0_state = received_data.btn[0]; // 現在のボタンの状態を次回の比較用に保存する
        }

        // 開発環境の画面に計算された各モーターの出力値を表示する
        printf("DEBUG: PWM Output -> RF:%d, RR:%d, LF:%d, LR:%d\n", (int)rf, (int)rr, (int)lf, (int)lr);

        // 決定した値を直接出力するのではなく目標値として設定し実際の滑らかな加減速は別ループに任せる
        set_target_crawler_drive_4ch(rf, rr, lf, lr);

        // 受信したレバーの方向の数値を画面に出力する
        printf("Direction: %d | Buttons: ", received_data.dir);

        // リモコンにある8個のボタンの状態を順番に画面に出力する
        for (int i = 0; i < 8; i++) {
            printf("[%d]", received_data.btn[i]);
        }
        printf("\n");
    } else {
        ESP_LOGW(TAG, "データサイズ不一致: 受信=%d, 想定=%d", len, sizeof(controller_data_t));
    }
}

// 通信機能および周辺機能全体の初期化を行う関数
void wifi_now_init(void) {
    // データの保存領域であるフラッシュメモリを初期化する
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // エラーが起きていないか確認する
    ESP_ERROR_CHECK(ret);

    // ネットワーク基盤を初期化しワイファイを子機モードで起動する
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 機器間通信であるイーエスピーナウを初期化する
    ESP_ERROR_CHECK(esp_now_init());
    
    // データを受信したときに動作する関数を登録する
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    // ステータスを表示するためのLEDの初期化を行う
    led_init();

    // 20ミリ秒ごとにLEDの色更新関数を呼び出すための定期タイマーを作成して開始する
    const esp_timer_create_args_t timer_args = {
        .callback = &led_update_timer_callback,
        .name = "led_update_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &led_update_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(led_update_timer, 20000)); // 20000マイクロ秒は20ミリ秒を意味する

    ESP_LOGI(TAG, "受信待機中（LEDステータス監視有効）...");
}

// 設計上の要件に合わせて用意されている空の関数で内部の処理はない
void send_data_to_main(controller_data_t *data) {
    // この関数は使用しない
}

// コントローラーとの通信が切断されずに維持されているかを確認する関数
bool get_controller_connection_status(void) {
    int64_t current_time = esp_timer_get_time();
    // 最後にデータを受け取ってからの経過時間が2秒未満であれば接続中とみなして真を返す
    return (current_time - last_packet_time) < 2000000;
}

// 最後に通信したときの電波強度を外部に返す関数
int get_controller_rssi(void) {
    return last_rssi;
}
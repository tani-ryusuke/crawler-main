idf_component_register(SRCS "uart_to_camera.c"
                            "main.c"
                            "esp_now_handler.c"
                            "main_PWM.c"
                            "buzzer_cmd.c"
                            "pir_sensor.c"
                           
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES led_strip driver esp_wifi nvs_flash esp_timer esp_http_server)
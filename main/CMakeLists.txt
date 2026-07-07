idf_component_register(SRCS "main.c"
                            "esp_now_handler.c"
                            "main_PWM.c"
                           
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES led_strip driver esp_wifi nvs_flash esp_timer esp_http_server)
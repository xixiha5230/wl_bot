idf_component_register(
    SRCS
        "app_main.c"
        "board.c"
        "wifi_net.c"
        "http_server.c"
        "robot_state.c"
        "sensors.c"
        "servo_sts.c"
        "motor_foc.cpp"
        "robot_control.cpp"
        "ota.c"
        "ws_server.c"
        "console.c"
    INCLUDE_DIRS "."
    REQUIRES
        console
        driver
        esp_adc
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_uart
        esp_event
        esp_http_client
        esp_netif
        esp_timer
        esp_http_server
        esp_wifi
        app_update
        nvs_flash
        espressif__esp_simplefoc
)

# Wi-Fi credentials come from the repo-root .env file (see .env.example) and/or
# the environment at configure time, so they never have to live in the
# repository, e.g.
#   WLROBOT_WIFI_SSID=my-net WLROBOT_WIFI_PASSWORD=secret tools/ota.sh
# Real environment variables win over .env; anything unset falls back to the
# default in wifi_net.c.
set(_env_file "${CMAKE_CURRENT_LIST_DIR}/../.env")
if(EXISTS "${_env_file}")
    file(STRINGS "${_env_file}" _env_lines)
    foreach(_line IN LISTS _env_lines)
        if(_line MATCHES "^([A-Za-z_][A-Za-z0-9_]*)=(.*)$")
            set(_name "${CMAKE_MATCH_1}")
            if(NOT DEFINED ENV{${_name}})
                set(ENV{${_name}} "${CMAKE_MATCH_2}")
            endif()
        endif()
    endforeach()
endif()

if(DEFINED ENV{WLROBOT_WIFI_SSID})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        "WIFI_STA_SSID=\"$ENV{WLROBOT_WIFI_SSID}\"")
endif()
if(DEFINED ENV{WLROBOT_WIFI_PASSWORD})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        "WIFI_STA_PASSWORD=\"$ENV{WLROBOT_WIFI_PASSWORD}\"")
endif()
if(DEFINED ENV{WLROBOT_AP_SSID})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        "WIFI_AP_SSID=\"$ENV{WLROBOT_AP_SSID}\"")
endif()
if(DEFINED ENV{WLROBOT_AP_PASSWORD})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        "WIFI_AP_PASSWORD=\"$ENV{WLROBOT_AP_PASSWORD}\"")
endif()

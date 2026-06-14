#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "secrets.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_timer.h" // ESP-IDF high-resolution timer API for esp_timer_get_time()

static const char *TAG = "C6_WIFI";

//--------------------------------
// UART settings
// Wiring:
// ESP32-S3 TX -> ESP32-C6 RX
// ESP32-S3 RX -> ESP32-C6 TX
// ESP32-S3 GND -> ESP32-C6 GND
//--------------------------------
#define TELEMETRY_UART_PORT UART_NUM_1
#define TELEMETRY_UART_BAUD_RATE 115200

// Verify that the UART pins are correct for your ESP32-C6 board before wiring
// For now they are firmware placeholders.
#define TELEMETRY_UART_TX_PIN 5
#define TELEMETRY_UART_RX_PIN 4

#define UART_RX_BUFFER_SIZE 1024
#define UART_LINE_BUFFER_SIZE 256

//--------------------------------
// Shared telemetry state.
// For now it starts with fake values, later it will be updated with real telemetry from the UART connection to the ESP32-S3.
//--------------------------------  
typedef struct {
    bool armed;  // Indicates whether the flight controller is armed or disarmed. C6 telemetry must not make decision to arm or disarm the flight controller, it only reports the state.
    char mode[16]; // Flight mode text received from the flight controller. (i.e. "DISARMED", "UART_TEST", "STABILIZE"). The buffer is intentionally small to avoid memory issues.
    float vbat; // Battery voltage in volts received from the flight controller. For now it is a test value, later it will be updated from the FC power-monitoring.
    int counter; //Monotonic counter that increments each time the telemetry is updated. Helps to verify that the telemetry is being updated and not stale.
    int motors[4]; // Motor values received from the FC for display/debugging.
    uint32_t last_packet_ms; //Timestamp of the last telemetry packet received from the FC. Unit: milliseconds since boot. Helps dashboard to detect stale telemetry.
    uint32_t valid_packets; // Number of telemetry packets that were successfully parsed and considered valid. Helps to monitor telemetry reliability.
    uint32_t invalid_packets; // Number of telemetry packets that were received but failed parsing or were considered invalid. Rising value may indicate protocol mismatch, baud-rate mismatch, or other communication issues.
} telemetry_state_t;

static telemetry_state_t latest_telemetry = {
    // Safe default state is disarmed.
    .armed = false, 
    .mode = "NO_LINK",
    .vbat = 0.0f,
    .counter = 0,
    .motors = {0, 0, 0, 0},

    .last_packet_ms = 0, // Zero means no packets received yet.
    .valid_packets = 0, // Diagnostic counters start from zero on every boot.
    .invalid_packets = 0
};

// Put your Wi-Fi details here locally.
// Do not paste your real password into chat.

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t wifi_event_group;
static int retry_count = 0;
static const int max_retry_count = 10;

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi station started. Connecting...");
        esp_wifi_connect();
    }

    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *disconnected =
        (wifi_event_sta_disconnected_t *) event_data;

    ESP_LOGW(TAG, "Disconnected. Reason code: %d", disconnected->reason);

    if (retry_count < max_retry_count) {
        retry_count++;
        ESP_LOGW(TAG, "Retrying %d/%d...", retry_count, max_retry_count);
        esp_wifi_connect();
    } else {
        ESP_LOGE(TAG, "Failed to connect after maximum retries.");
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
      }
    }

    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;

        retry_count = 0;

        ESP_LOGI(TAG, "Wi-Fi connected.");
        ESP_LOGI(TAG, "IP address: " IPSTR, IP2STR(&event->ip_info.ip));

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL,
            NULL
        )
    );

    wifi_config_t wifi_config = {0};

    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init finished.");

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi SSID: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Unexpected Wi-Fi event.");
    }
}

static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    // Calculate telemetry link health. UART parser records latest_telemetry.last_packet_ms when a valid telemetry packet is received. 
    // The HTTP API compares the current time with the last packet timestamp to determine if the telemetry link is healthy or stale or down.
    // Display/diagnostic only, not used for any control decisions.
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL); // Get current time in milliseconds since boot.
    uint32_t packet_age_ms = 0; // Age of the last telemetry packet in milliseconds. If no packets have been received yet, it will remain zero.
    const char *link_status = "NO_LINK"; // Default status if no packets have been received yet.
    const uint32_t telemetry_timeout_ms = 3000; // Timeout for considering telemetry link as down.

    // If at least one telemetry packet has been received, calculate the age of the last packet.
    if (latest_telemetry.last_packet_ms > 0) { 
        packet_age_ms = now_ms - latest_telemetry.last_packet_ms;

        if (packet_age_ms < telemetry_timeout_ms) {
            link_status = "OK";
        } else {
            link_status = "STALE";
        }
    }

    char response[256];
    
    // Build the JSON response for the browser dashboard. It includes the latest telemetry state and the link health status.
    snprintf(
        response,
        sizeof(response),
        "{"
        "\"armed\":%s,"
        "\"mode\":\"%s\","
        "\"vbat\":%.2f,"
        "\"counter\":%d,"
        "\"motors\":[%d,%d,%d,%d],"
        "\"link_status\":\"%s\","
        "\"packet_age_ms\":%lu,"
        "\"valid_packets\":%lu,"
        "\"invalid_packets\":%lu"
        "}",
        latest_telemetry.armed ? "true" : "false",
        latest_telemetry.mode,
        latest_telemetry.vbat,
        latest_telemetry.counter,
        latest_telemetry.motors[0],
        latest_telemetry.motors[1],
        latest_telemetry.motors[2],
        latest_telemetry.motors[3],
        link_status,
        (unsigned long)packet_age_ms,
        (unsigned long)latest_telemetry.valid_packets,
        (unsigned long)latest_telemetry.invalid_packets
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *html =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset='utf-8'>"
        "<title>C6 Telemetry Bridge</title>"
        "<style>"
        "body{font-family:Arial;background:#111;color:#eee;margin:30px;}"
        ".card{background:#222;border:1px solid #444;padding:16px;margin-bottom:16px;border-radius:8px;}"
        ".label{color:#aaa;font-size:14px;}"
        ".value{font-size:26px;font-weight:bold;}"
        ".ok{color:#55ff88;}"
        ".bad{color:#ff5555;}"
        "pre{background:#000;padding:12px;border-radius:6px;overflow-x:auto;}"
        "</style>"
        "</head>"
        "<body>"
        "<h1>ESP32-C6 Telemetry Bridge</h1>"

        "<div class='card'>"
        "<div class='label'>Status</div>"
        "<div id='status' class='value'>Loading...</div>"
        "</div>"

        "<div class='card'>"
        "<div class='label'>Battery Voltage</div>"
        "<div id='vbat' class='value'>-- V</div>"
        "</div>"

        "<div class='card'>"
        "<div class='label'>Motors</div>"
        "<div id='motors' class='value'>--</div>"
        "</div>"

        "<div class='card'>"
        "<div class='label'>Counter</div>"
        "<div id='counter' class='value'>--</div>"
        "</div>"

        "<div class='card'>"
        "<div class='label'>Raw JSON</div>"
        "<pre id='raw'>--</pre>"
        "</div>"

        "<script>"
        "async function updateTelemetry(){"
        "try{"
        "const r=await fetch('/telemetry');"
        "const d=await r.json();"
        "const s=document.getElementById('status');"
        "s.textContent=d.armed?'ARMED - '+d.mode:'DISARMED';"
        "s.className=d.armed?'value bad':'value ok';"
        "document.getElementById('vbat').textContent=d.vbat.toFixed(2)+' V';"
        "document.getElementById('motors').textContent=d.motors.join(', ');"
        "document.getElementById('counter').textContent=d.counter;"
        "document.getElementById('raw').textContent=JSON.stringify(d,null,2);"
        "}catch(e){"
        "document.getElementById('status').textContent='Connection error';"
        "document.getElementById('status').className='value bad';"
        "}"
        "}"
        "setInterval(updateTelemetry,500);"
        "updateTelemetry();"
        "</script>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    esp_err_t result = httpd_start(&server, &config);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(result));
        return;
    }

    httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL
  };

    httpd_uri_t telemetry_uri = {
    .uri = "/telemetry",
    .method = HTTP_GET,
    .handler = telemetry_get_handler,
    .user_ctx = NULL
  };

  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &telemetry_uri));

  ESP_LOGI(TAG, "HTTP server started.");
  ESP_LOGI(TAG, "Dashboard endpoint: /");
  ESP_LOGI(TAG, "Telemetry endpoint: /telemetry");
}

static void telemetry_uart_task(void *arg)
{
    uint8_t rx_byte;
    char line_buffer[UART_LINE_BUFFER_SIZE];
    int line_pos = 0;

    ESP_LOGI(TAG, "UART telemetry task started.");
    ESP_LOGI(TAG, "Waiting for telemetry lines on UART%d at %d baud.",
             TELEMETRY_UART_PORT, TELEMETRY_UART_BAUD_RATE);

    while (1) {
        int len = uart_read_bytes(
            TELEMETRY_UART_PORT,
            &rx_byte,
            1,
            pdMS_TO_TICKS(100)
        );

        if (len > 0) {
            if (rx_byte == '\n') {
                line_buffer[line_pos] = '\0';

                ESP_LOGI(TAG, "UART line received: %s", line_buffer);

                // Simple controlled-format parser for H2 test telemetry.
                // Expected example:
                // {"source":"h2","armed":true,"mode":"UART_TEST","vbat":12.34,"counter":123}
                char parsed_mode[16] = {0};
                float parsed_vbat = 0.0f;
                int parsed_counter = 0;
                
                bool parsed_armed = strstr(line_buffer, "\"armed\":true") != NULL;
                
                int parsed_fields = sscanf(
                        line_buffer,
                        "{\"source\":\"h2\",\"armed\":%*[^,],\"mode\":\"%15[^\"]\",\"vbat\":%f,\"counter\":%d}",
                        parsed_mode,
                        &parsed_vbat,
                        &parsed_counter
                    );
                    
                    if (parsed_fields == 3) {
                        latest_telemetry.armed = parsed_armed; // Telemetry line was received and matched the expected format, so we update the latest telemetry state.
                        //Copy the parsed mode into the latest telemetry state and avoid buffer overflow.
                        strncpy( 
                            latest_telemetry.mode,
                            parsed_mode,
                            sizeof(latest_telemetry.mode) - 1
                        );
                        latest_telemetry.mode[sizeof(latest_telemetry.mode) - 1] = '\0';
                        latest_telemetry.vbat = parsed_vbat; //Store the parsed battery voltage into the latest telemetry state.
                        latest_telemetry.counter = parsed_counter; //Store the parsed counter into the latest telemetry state.
                        latest_telemetry.motors[0] = 1100; // For now, we use test values for motors. Later, these will be updated from real telemetry data from the FC.
                        latest_telemetry.motors[1] = 1110;
                        latest_telemetry.motors[2] = 1120;
                        latest_telemetry.motors[3] = 1130;
                        latest_telemetry.last_packet_ms = (uint32_t)(esp_timer_get_time() / 1000ULL); // Store the timestamp of the last telemetry packet received from the FC. esp_timer_get_time() returns time in microseconds, so we divide by 1000 to convert to milliseconds.
                        latest_telemetry.valid_packets++; // Increment the count of valid telemetry packets received.

                        ESP_LOGI(TAG,
                            "Parsed telemetry: armed=%d mode=%s vbat=%.2f counter=%d valid=%lu",
                            latest_telemetry.armed,
                            latest_telemetry.mode,
                            latest_telemetry.vbat,
                            latest_telemetry.counter,
                            (unsigned long)latest_telemetry.valid_packets);
                    } else {
                        latest_telemetry.invalid_packets++; // Increment the count of invalid telemetry packets received.
                        // Log a warning if the telemetry line could not be parsed correctly.
                        ESP_LOGW(TAG,  
                             "Could not parse telemetry line. invalid=%lu line=%s",
                              (unsigned long)latest_telemetry.invalid_packets,
                              line_buffer);
                    }

                line_pos = 0;
            }
            else if (rx_byte != '\r') {
                if (line_pos < UART_LINE_BUFFER_SIZE - 1) {
                    line_buffer[line_pos++] = (char)rx_byte;
                } else {
                    ESP_LOGW(TAG, "UART line too long. Dropping buffer.");
                    line_pos = 0;
                }
            }
        }
    }
}

static void telemetry_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = TELEMETRY_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Initializing telemetry UART%d...", TELEMETRY_UART_PORT);

    ESP_ERROR_CHECK(uart_driver_install(
        TELEMETRY_UART_PORT,
        UART_RX_BUFFER_SIZE,
        0,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(
        TELEMETRY_UART_PORT,
        &uart_config
    ));

    ESP_ERROR_CHECK(uart_set_pin(
        TELEMETRY_UART_PORT,
        TELEMETRY_UART_TX_PIN,
        TELEMETRY_UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_LOGI(TAG, "Telemetry UART initialized.");
    ESP_LOGI(TAG, "UART TX pin: GPIO%d", TELEMETRY_UART_TX_PIN);
    ESP_LOGI(TAG, "UART RX pin: GPIO%d", TELEMETRY_UART_RX_PIN);
}

void app_main(void)
{
    ESP_LOGI(TAG, "C6 telemetry bridge Wi-Fi test starting...");

    esp_err_t nvs_result = nvs_flash_init();

    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }

    ESP_ERROR_CHECK(nvs_result);

    wifi_init_sta();

    telemetry_uart_init();

    xTaskCreate(
        telemetry_uart_task,
        "telemetry_uart_task",
        4096,
        NULL,
        5,
        NULL
    );
    
    start_webserver();

    int counter = 0;

    while (1) {
        ESP_LOGI(TAG, "Wi-Fi telemetry bridge alive. Counter: %d", counter++);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "secrets.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

static const char *TAG = "C6_WIFI";

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
    static int counter = 0;
    float vbat = 12.60f - ((counter % 100) * 0.01f);
    char response[256];

    snprintf(
        response,
        sizeof(response),
        "{"
            "\"armed\":false,"
            "\"mode\":\"DISARMED\","
            "\"vbat\":%.2f,"
            "\"counter\":%d,"
            "\"motors\":[1000,1000,1000,1000]"
        "}",
        vbat,
        counter
    );

    counter++;

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
    
    start_webserver();

    int counter = 0;

    while (1) {
        ESP_LOGI(TAG, "Wi-Fi telemetry bridge alive. Counter: %d", counter++);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
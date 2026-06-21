// Standard C library headers.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// FreeRTOS services used by the application.
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

// ESP-IDF peripheral and platform APIs.
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "mdns.h"

// Local project configuration.
#include "secrets.h"

// Logging tag shown at the beginning of this module's ESP-IDF log lines. It helps to identify which module generated the log message.
static const char *TAG = "C6_WIFI";

//--------------------------------
// UART settings
// Wiring:
// ESP32-S3 TX -> ESP32-C6 RX
// ESP32-S3 RX -> ESP32-C6 TX
// ESP32-S3 GND -> ESP32-C6 GND
//--------------------------------

// The H2 is only a temp telemetry source used during development. The final telemetry source will be the flight controller (FC).
#define TELEMETRY_UART_PORT UART_NUM_1
#define TELEMETRY_UART_BAUD_RATE 115200

// Current ESP32-C6 UART pin allocation. GPIO4 receives telemetry from the sender, GPIO5 is reserved as the C6 telemetry TX output for future bidirectional communication.
#define TELEMETRY_UART_TX_PIN 5
#define TELEMETRY_UART_RX_PIN 4

//UART driver receive buffer and application line-buffer sizes.
#define UART_RX_BUFFER_SIZE 1024
#define UART_LINE_BUFFER_SIZE 256

//--------------------------------
// Shared telemetry state.
//--------------------------------

// Represents the most recently validated telemetry state.
typedef struct {
    bool armed;  // Indicates whether the flight controller is armed or disarmed.
    char mode[16]; // Flight mode text received from the flight controller. (i.e. "DISARMED", "UART_TEST", "STABILIZE"). The buffer is intentionally small to avoid memory issues.
    float vbat; // Battery voltage in volts received from the flight controller. For now it is a test value, later it will be updated from the FC power-monitoring.
    int counter; //Monotonic counter that increments each time the telemetry is updated. Helps to verify that the telemetry is being updated and not stale.
    int motors[4]; // Motor values received from the FC for display/debugging.
    uint32_t last_packet_ms; //Timestamp of the last telemetry packet received from the FC. Unit: milliseconds since boot. Helps dashboard to detect stale telemetry.
    uint32_t valid_packets; // Number of telemetry packets that were successfully parsed and considered valid. Helps to monitor telemetry reliability.
    uint32_t invalid_packets; // Number of telemetry packets that were received but failed parsing or were considered invalid. Rising value may indicate protocol mismatch, baud-rate mismatch, or other communication issues.
} telemetry_state_t;

// Safe startup state before any valid telemetry packet arrives.
static telemetry_state_t latest_telemetry = {
    .armed = false, 
    .mode = "NO_LINK",
    .vbat = 0.0f,
    .counter = 0,
    .motors = {0, 0, 0, 0},
    .last_packet_ms = 0, // Zero means no packets received yet.
    .valid_packets = 0, // Diagnostic counters start from zero on every boot.
    .invalid_packets = 0
};

//--------------------------------
// Wi-Fi connection management
// Put Wi-Fi details here locally.
//--------------------------------

// FreeRTOS event-group bits used to communicate Wi-Fi state.
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Stable mDNS hostname used on the local network. Device should become reachable as: http://c6-telemetry.local/
#define MDNS_HOSTNAME "c6-telemetry"
// Friendly service name shown by mDNS/DNS-SD discovery tools.
#define MDNS_INSTANCE_NAME "C6 Drone Telemetry Bridge"

// Delay between Wi-Fi reconnect attempt groups. esp_timer uses microseconds, therefore thhis value represents 5 seconds.
#define WIFI_RECONNECT_DELAY_US (5ULL * 1000ULL * 1000ULL) 

// Shared Wi-Fi synchronization object. NULL means it has not yet been created.
static EventGroupHandle_t wifi_event_group = NULL;
// Number of immediate reconnect attempts made in the current retry cycle.
static int retry_count = 0;
// Maximum number of immediate retries before scheduling a delayed retry.
static const int max_retry_count = 10; 
// One-shot ESP timer to delay the next Wi-Fi reconnect attempt to avoid blcking the ESP-IDF event handler task.
static esp_timer_handle_t wifi_reconnect_timer = NULL;

// Global HTTP server handle. Must be global because both the Wi-Fi event handler and the HTTP lifecycle functions need access to the same server instance.
// NULL means that the HTTP server is currently stopped.
static httpd_handle_t http_server = NULL;

// Tracks whether the mDNS subsystem has already been initialized. Should be initialized only once and after initialization it tracks the standard Wi-Fi station interface and reacts to interface state changes through the ESP-IDF networking event system.
static bool mdns_initialized = false;

static void start_webserver(void);
static void stop_webserver(void);
// Initialize the local mDNS hostname and advertises the HTTP service.
static void start_mdns_service(void);

// The callback starts a new connection attempt after the configured delay period.
static void wifi_reconnect_timer_callback(void* arg){
    //The callback argument is not used, but we include it to match the expected signature for esp_timer callbacks.
    (void)arg;
    retry_count = 0; // Reset the retry count for the new connection attempt cycle.

    ESP_LOGI(TAG, "Wi-Fi reconnect delay expired. Starting a new retry cycle."); //esp_wifi_connect() starts one connection attempt. Whatever the outcome it is handled by the Wi-Fi event handler.
    esp_err_t err = esp_wifi_connect();
    
    // Do not abort the application for a recoverable Wi-Fi connection failure. Record the error and continue.
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start delayed Wi-Fi connection attempt: %s", esp_err_to_name(err));
    } 
}

//Initializes the mDNS responder for the telemetry bridge. The function is idempotent: repeated calls return immediately after successful initialization.
// This is important because GOT_IP can occur multiple times during Wi-Fi reconnect cycles.
static void start_mdns_service(void)
{
    // Avoid initializing the mDNS component more than once.
    if (mdns_initialized) {
        ESP_LOGI(
            TAG,
            "mDNS is already initialized at http://%s.local/",
            MDNS_HOSTNAME
        );
        return;
    }

    // Start the mDNS responder. This creates the internal mDNS task and registers the required networking event handling.
    esp_err_t result = mdns_init();

    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to initialize mDNS: %s",
            esp_err_to_name(result)
        );
        return;
    }

    // Set the hostname that other devices will resolve. Example: MDNS_HOSTNAME = "c6-telemetry" becomes: c6-telemetry.local
    result = mdns_hostname_set(MDNS_HOSTNAME);

    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to set mDNS hostname: %s",
            esp_err_to_name(result)
        );

        // Release the partially initialized mDNS subsystem.
        mdns_free();
        return;
    }

    // Set a human-readable device name used by service browsers.
    result = mdns_instance_name_set(MDNS_INSTANCE_NAME);

    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to set mDNS instance name: %s",
            esp_err_to_name(result)
        );

        mdns_free();
        return;
    }

    // Advertise the embedded HTTP server through DNS Service Discovery.
    // Service type: _http
    // Protocol:     _tcp
    // Port:         80
    result = mdns_service_add(
        NULL,
        "_http",
        "_tcp",
        80,
        NULL,
        0
    );

    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to advertise HTTP service through mDNS: %s",
            esp_err_to_name(result)
        );

        mdns_free();
        return;
    }

    // Mark initialization complete only after every required step succeeds.
    mdns_initialized = true;

    ESP_LOGI(
        TAG,
        "mDNS started. Dashboard URL: http://%s.local/",
        MDNS_HOSTNAME
    );
}

//Handles asynchronous Wi-Fi and IP events generated by ESP-IDF. This function must remain short and non-blocking because it executes in the system event-loop context.
static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
)
{
    // The handler argument is not currently used.
    (void)arg;

    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START) {

        ESP_LOGI(TAG, "Wi-Fi station started. Connecting...");

        // Clear old state before the first connection attempt.
        xEventGroupClearBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT
        );

        retry_count = 0;

        esp_err_t err = esp_wifi_connect();

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Failed to start initial Wi-Fi connection: %s",
                esp_err_to_name(err)
            );
        }
    }

    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {

        wifi_event_sta_disconnected_t *disconnected =
            (wifi_event_sta_disconnected_t *)event_data;

        // The station no longer has a usable Wi-Fi connection. Clear the connected bit immediately so application state does not continue to report a valid network connection.
        xEventGroupClearBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );

        // Stop the HTTP server because all the active TCP sessions belong to the disconnected network interface and must no longer be considered valid. A fresh server instance will be created when the station regains a valid IP address.
        stop_webserver();

        ESP_LOGW(
            TAG,
            "Wi-Fi disconnected. Reason code: %d",
            disconnected->reason
        );

        if (retry_count < max_retry_count) {
            // Perform a limited number of immediate retries. These handle short interruptions without introducing an unnecessary five-second delay.
            retry_count++;

            ESP_LOGW(
                TAG,
                "Immediate Wi-Fi retry %d/%d...",
                retry_count,
                max_retry_count
            );

            esp_err_t err = esp_wifi_connect();

            if (err != ESP_OK) {
                ESP_LOGE(
                    TAG,
                    "Failed to start immediate Wi-Fi retry: %s",
                    esp_err_to_name(err)
                );
            }
        } else {
            // The immediate retry group has been exhausted. WIFI_FAIL_BIT allows wifi_init_sta() to finish its initial blocking wait, while the background recovery system continues.
            xEventGroupSetBits(
                wifi_event_group,
                WIFI_FAIL_BIT
            );

            ESP_LOGW(
                TAG,
                "Immediate retries exhausted. "
                "Scheduling another retry cycle in 5 seconds."
            );

            // Avoid starting the one-shot timer more than once.
            if (!esp_timer_is_active(wifi_reconnect_timer)) {
                esp_err_t err = esp_timer_start_once(
                    wifi_reconnect_timer,
                    WIFI_RECONNECT_DELAY_US
                );

                if (err != ESP_OK) {
                    ESP_LOGE(
                        TAG,
                        "Failed to schedule Wi-Fi reconnect timer: %s",
                        esp_err_to_name(err)
                    );
                }
            }
        }
    }

    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        // A valid IP address means the station is operational again.
        retry_count = 0;

        // Cancel any delayed reconnect that may still be pending.
        if (esp_timer_is_active(wifi_reconnect_timer)) {
            esp_timer_stop(wifi_reconnect_timer);
        }

        // Remove stale failure state and publish the connected state.
        xEventGroupClearBits(
            wifi_event_group,
            WIFI_FAIL_BIT
        );

        xEventGroupSetBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );

        ESP_LOGI(TAG, "Wi-Fi connected.");
        ESP_LOGI(
            TAG,
            "IP address: " IPSTR,
            IP2STR(&event->ip_info.ip)
        );

        // Start or restore dashboard availability after the network recovery.
        start_webserver();

        // Initialize mDNS after the station has a valid IP address. On later Wi-Fi reconnects this function safely returns because the mDNS subsystem has already been initialized.
        start_mdns_service();
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    // Validate that the FREERTOS event group was created successfully. Failure indicates a serious memory allocation issue that must be addressed before continuing.
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi event group.");
        abort();
    }

    // Configure the one-shot timer used for the delayed Wi-Fi reconnection.
    const esp_timer_create_args_t reconnect_timer_args = {
        .callback = &wifi_reconnect_timer_callback, //Function to call when the timer expires.
        .arg = NULL, //No argument is needed for this callback.
        .name = "wifi_reconnect" //Name for debugging purposes.
    };
    
    ESP_ERROR_CHECK(
        esp_timer_create(
            &reconnect_timer_args,
            &wifi_reconnect_timer
        )
    );

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
        ".link-ok{color:#1b8f3a;font-weight:bold;}"
        ".link-stale{color:#c0392b;font-weight:bold;}"
        ".link-no-link{color:#7f8c8d;font-weight:bold;}"
        ".link-unknown{color:#d68910;font-weight:bold;}"
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

        // Telemetry link health and diagnostics, JavaScript will update these values based on the latest telemetry received from the flight controller.  
        "<p><strong>Link Status:</strong> "
        "<span id=\"linkStatus\" class=\"link-unknown\">UNKNOWN</span></p>"

        "<p><strong>Packet Age:</strong> "
        "<span id=\"packetAge\">--</span> ms</p>"

        "<p><strong>Valid Packets:</strong> "
        "<span id=\"validPackets\">0</span></p>"

        "<p><strong>Invalid Packets:</strong> "
        "<span id=\"invalidPackets\">0</span></p>"

        "<div class='card'>"
        "<div class='label'>Raw JSON</div>"
        "<pre id='raw'>--</pre>"
        "</div>"
        "<script>"
        "async function updateTelemetry(){"
        "try{"
        "const r=await fetch('/telemetry');"
        "const d=await r.json();"
        // Update the main aircraft status.
        "const s=document.getElementById('status');"
        "s.textContent=d.armed?'ARMED - '+d.mode:'DISARMED';"
        "s.className=d.armed?'value bad':'value ok';"
        // Update the main telemetry values.
        "document.getElementById('vbat').textContent=d.vbat.toFixed(2)+' V';"
        "document.getElementById('motors').textContent=d.motors.join(', ');"
        "document.getElementById('counter').textContent=d.counter;"
        // Display the telemetry link health info returned by the C6 API. The nullish coalescing operator (??) is used to handle cases where the telemetry API may not return a value yet.
        "document.getElementById('packetAge').textContent=(d.packet_age_ms ?? '--');"
        "document.getElementById('validPackets').textContent=(d.valid_packets ?? 0);"
        "document.getElementById('invalidPackets').textContent=(d.invalid_packets ?? 0);"
        // Update the link status with appropriate color coding based on the telemetry API response.
        "const linkElement=document.getElementById('linkStatus');"
        "linkElement.textContent=d.link_status ?? 'UNKNOWN';"

        "switch(d.link_status){"
        "case 'OK':"
          "linkElement.className='link-ok';"
          "break;"
           
        "case 'STALE':"
          "linkElement.className='link-stale';"
          "break;"

        "case 'NO_LINK':"
          "linkElement.className='link-no-link';"
          "break;"

        "default:"
          "linkElement.className='link-unknown';"
          "break;"
        "}"
        // Display the raw JSON telemetry data for debugging purposes. This allows developers to see the exact data structure being sent from the C6 telemetry bridge.
        "document.getElementById('raw').textContent=JSON.stringify(d,null,2);"
        "}catch(e){"
        // If the HTTP request fails (e.g., due to network issues or the C6 telemetry bridge being down), display a connection error message in red.
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

// Starts the embedded HTTP server and registers all dashboard endpoints. The function is idempotent: if the server is already running, it returns without creating a second HTTP server instance.
static void start_webserver(void)
{
    // Prevent duplicate server instances. Starting two servers on the same TCP port would fail and could also leave the application state inconsistent.
    if (http_server != NULL) {
        ESP_LOGW(TAG, "HTTP server is already running.");
        return;
    }

    // Start from ESP-IDF's recommended default HTTP server configuration.
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(
        TAG,
        "Starting HTTP server on port %d",
        config.server_port
    );

    // Store the server handle in the global http_server variable. This allows the Wi-Fi event handler to stop the server later.
    esp_err_t result = httpd_start(
        &http_server,
        &config
    );

    if (result != ESP_OK) {
        // Ensure the global state remains accurate after startup failure.
        http_server = NULL;

        ESP_LOGE(
            TAG,
            "Failed to start HTTP server: %s",
            esp_err_to_name(result)
        );

        return;
    }

    // Dashboard root endpoint.
    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = NULL
    };

    // Machine-readable telemetry endpoint.
    const httpd_uri_t telemetry_uri = {
        .uri = "/telemetry",
        .method = HTTP_GET,
        .handler = telemetry_get_handler,
        .user_ctx = NULL
    };

    // Register the root dashboard endpoint. Do not use ESP_ERROR_CHECK here because registration failure is recoverable. We cleanly stop the partially configured server instead.
    result = httpd_register_uri_handler(
        http_server,
        &root_uri
    );

    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to register dashboard endpoint: %s",
            esp_err_to_name(result)
        );

        httpd_stop(http_server);
        http_server = NULL;
        return;
    }

    // Register the telemetry API endpoint.
    result = httpd_register_uri_handler(
        http_server,
        &telemetry_uri
    );

    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to register telemetry endpoint: %s",
            esp_err_to_name(result)
        );

        httpd_stop(http_server);
        http_server = NULL;
        return;
    }

    ESP_LOGI(TAG, "HTTP server started.");
    ESP_LOGI(TAG, "Dashboard endpoint: /");
    ESP_LOGI(TAG, "Telemetry endpoint: /telemetry");
}

// Stops the embedded HTTP server and closes all active HTTP connections. This function is safe to call even when the server is already stopped.
static void stop_webserver(void)
{
    // No server is currently active.
    if (http_server == NULL) {
        ESP_LOGI(TAG, "HTTP server is already stopped.");
        return;
    }

    ESP_LOGI(TAG, "Stopping HTTP server...");

    // httpd_stop() blocks until the HTTP server task terminates. It also closes open sessions and releases server resources.
    esp_err_t result = httpd_stop(http_server);

    if (result != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to stop HTTP server cleanly: %s",
            esp_err_to_name(result)
        );

        // Even after an error, clear the handle so the application does not treat the old server instance as usable.
        http_server = NULL;
        return;
    }

    // Mark the server as stopped. A future IP_EVENT_STA_GOT_IP event may now start a fresh instance.
    http_server = NULL;

    ESP_LOGI(TAG, "HTTP server stopped.");
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
                        // Copy the parsed mode into the latest telemetry state and avoid buffer overflow.
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
    
    int counter = 0;

    while (1) {
        ESP_LOGI(TAG, "Wi-Fi telemetry bridge alive. Counter: %d", counter++);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
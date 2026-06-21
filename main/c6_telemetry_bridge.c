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
#include "cJSON.h"

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
// Telemetry protocol contract.
//--------------------------------

// Version of the telemetry packet format currently accepted by the bridge.
// Increment this value only when the packet structure changes in a way that is not backward-compatible.
#define TELEMETRY_PROTOCOL_VERSION 1
// Expected logical source identifier.
// Prevents arbitrary JSON data received on UART from being treated as valid flight-controller telemetry.
#define TELEMETRY_EXPECTED_SOURCE "flight_controller"
// Maximum accepted mode-name length, excluding the terminating '\0'.
// The telemetry_state_t buffer is 16 bytes, therefore the maximum safe payload length is 15 characters.
#define TELEMETRY_MODE_MAX_LENGTH 15
// Initial validation limits for battery voltage.
// These are broad electrical sanity limits, not battery-warning thresholds. The final limits will later depend on the selected battery configuration.
#define TELEMETRY_VBAT_MIN_VOLTS 0.0
#define TELEMETRY_VBAT_MAX_VOLTS 60.0
// Valid PWM-style motor-output range used during this development stage.
//These limits will be revisited when the final ESC protocol is selected.
#define TELEMETRY_MOTOR_MIN 800
#define TELEMETRY_MOTOR_MAX 2200

//--------------------------------
// Shared telemetry state.
//--------------------------------

// Represents the most recently validated telemetry state.
typedef struct {
    bool armed;  // Indicates whether the flight controller is armed or disarmed.
    char mode[16]; // Flight mode text received from the flight controller. (i.e. "DISARMED", "UART_TEST", "STABILIZE"). The buffer is intentionally small to avoid memory issues.
    float vbat; // Battery voltage in volts received from the flight controller. For now it is a test value, later it will be updated from the FC power-monitoring.
    uint32_t counter; // Monotonic counter that increments each time the telemetry is updated. Helps to verify that the telemetry is being updated and not stale.
    int motors[4]; // Motor values received from the FC for display/debugging.
    uint32_t last_packet_ms; // Timestamp of the last telemetry packet received from the FC. Unit: milliseconds since boot. Helps dashboard to detect stale telemetry.
    uint32_t valid_packets; // Number of telemetry packets that were successfully parsed and considered valid. Helps to monitor telemetry reliability.
    uint32_t invalid_packets; // Number of telemetry packets that were received but failed parsing or were considered invalid. Rising value may indicate protocol mismatch, baud-rate mismatch, or other communication issues.
} telemetry_state_t;

// Represents one completely parsed and validated telemetry packet.
// This is intentionally separate from telemetry_state_t:
//  telemetry_packet_t contains only data supplied by the sender.
//  telemetry_state_t also contains local bridge diagnostics such as packet timestamps and valid/invalid packet counters.
// The parser fills this temporary structure first. The shared dashboard state is updated only after the whole packet has passed validation.
typedef struct {
    // Telemetry protocol version declared by the sender.
    uint32_t protocol_version;

    // Monotonically increasing packet sequence number. This will later be used to detect duplicate, missing, or backwards packets.
    uint32_t sequence;

    // Armed state reported by the flight controller.
    bool armed;

    // Null-terminated flight-mode name. The array size allows up to TELEMETRY_MODE_MAX_LENGTH characters plus the required terminating null byte.
    char mode[TELEMETRY_MODE_MAX_LENGTH + 1];

    // Battery voltage reported by the flight controller, in volts.
    float vbat;

    // Motor outputs reported by the flight controller. These values are monitoring data only. The C6 does not generate or apply motor commands.
    int motors[4];
} telemetry_packet_t;

// Describes the exact result of parsing and validating one UART line.
// Keeping distinct error categories allows more useful logs, separate diagnostic counters later, easier protocol troubleshooting, clearer unit tests.
typedef enum {
    // Packet syntax, field types, and values are all valid.
    TELEMETRY_PARSE_OK = 0,

    // A null input pointer or output pointer was supplied.
    TELEMETRY_PARSE_INVALID_ARGUMENT,

    // Input was not syntactically valid JSON.
    TELEMETRY_PARSE_JSON_SYNTAX_ERROR,

    // The top-level JSON value was not an object.
    TELEMETRY_PARSE_ROOT_NOT_OBJECT,

    // One or more required fields were missing.
    TELEMETRY_PARSE_MISSING_FIELD,

    // A field existed but had the wrong JSON type. Example: "armed": "true" instead of "armed": true.
    TELEMETRY_PARSE_WRONG_TYPE,

    // protocol_version was not supported by this firmware.
    TELEMETRY_PARSE_UNSUPPORTED_VERSION,

    // source did not match TELEMETRY_EXPECTED_SOURCE.
    TELEMETRY_PARSE_INVALID_SOURCE,

    // sequence was negative, non-integral, or outside the supported range.
    TELEMETRY_PARSE_INVALID_SEQUENCE,

    // mode was empty, too long, or contained an invalid value.
    TELEMETRY_PARSE_INVALID_MODE,

    // Battery voltage was outside the broad sanity limits.
    TELEMETRY_PARSE_INVALID_VBAT,

    // motors was not a four-element array or contained invalid values.
    TELEMETRY_PARSE_INVALID_MOTORS
} telemetry_parse_result_t;

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

// Parses and validates one complete JSON telemetry line.
//--------------------------------
// On success:
// returns TELEMETRY_PARSE_OK,
// writes the validated packet into output_packet.

// On failure:
// returns a specific telemetry_parse_result_t value,
// does not modify the shared latest_telemetry state.
//--------------------------------
static telemetry_parse_result_t parse_telemetry_json(
    const char *json_text,
    telemetry_packet_t *output_packet
);

// Returns a readable diagnostic name for a parser result.
//  This keeps logging code simple and avoids scattering switch statements throughout the UART task.
static const char *telemetry_parse_result_to_string(
    telemetry_parse_result_t result
);


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
        "\"counter\":%lu,"
        "\"motors\":[%d,%d,%d,%d],"
        "\"link_status\":\"%s\","
        "\"packet_age_ms\":%lu,"
        "\"valid_packets\":%lu,"
        "\"invalid_packets\":%lu"
        "}",
        latest_telemetry.armed ? "true" : "false",
        latest_telemetry.mode,
        latest_telemetry.vbat,
        (unsigned long)latest_telemetry.counter,
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

// Checks whether a flight-mode name follows the protocol naming rules.
// Version 1 accepts: uppercase ASCII letters A-Z, digits 0-9, underscore, hyphen.
// Examples: MANUAL, STABILIZE, ALT_HOLD, FAILSAFE-1
static bool telemetry_mode_is_valid(const char *mode)
{
    if (mode == NULL) {
        return false;
    }

    size_t length = strlen(mode);

    // Reject empty names and strings that do not fit in the protocol buffer.
    if (length == 0 || length > TELEMETRY_MODE_MAX_LENGTH) {
        return false;
    }

    for (size_t index = 0; index < length; index++) {
        char character = mode[index];

        bool is_uppercase_letter =
            character >= 'A' && character <= 'Z';

        bool is_digit =
            character >= '0' && character <= '9';

        bool is_allowed_separator =
            character == '_' || character == '-';

        if (!is_uppercase_letter &&
            !is_digit &&
            !is_allowed_separator) {
            return false;
        }
    }

    return true;
}


//Converts parser-result values into stable diagnostic names.
// Returning constant strings avoids dynamic memory allocation in logging.
static const char *telemetry_parse_result_to_string(
    telemetry_parse_result_t result
)
{
    switch (result) {
        case TELEMETRY_PARSE_OK:
            return "OK";

        case TELEMETRY_PARSE_INVALID_ARGUMENT:
            return "INVALID_ARGUMENT";

        case TELEMETRY_PARSE_JSON_SYNTAX_ERROR:
            return "JSON_SYNTAX_ERROR";

        case TELEMETRY_PARSE_ROOT_NOT_OBJECT:
            return "ROOT_NOT_OBJECT";

        case TELEMETRY_PARSE_MISSING_FIELD:
            return "MISSING_FIELD";

        case TELEMETRY_PARSE_WRONG_TYPE:
            return "WRONG_TYPE";

        case TELEMETRY_PARSE_UNSUPPORTED_VERSION:
            return "UNSUPPORTED_VERSION";

        case TELEMETRY_PARSE_INVALID_SOURCE:
            return "INVALID_SOURCE";

        case TELEMETRY_PARSE_INVALID_SEQUENCE:
            return "INVALID_SEQUENCE";

        case TELEMETRY_PARSE_INVALID_MODE:
            return "INVALID_MODE";

        case TELEMETRY_PARSE_INVALID_VBAT:
            return "INVALID_VBAT";

        case TELEMETRY_PARSE_INVALID_MOTORS:
            return "INVALID_MOTORS";

        default:
            return "UNKNOWN_PARSE_RESULT";
    }
}


// Parses and validates one JSON telemetry packet.
/* Expected protocol-version-1 packet:
   {
 *   "protocol_version": 1,
 *   "source": "flight_controller",
 *   "sequence": 1234,
 *   "armed": true,
 *   "mode": "STABILIZE",
 *   "vbat": 15.82,
 *   "motors": [1100, 1110, 1120, 1130]
 * }
*/
// Design rule: The output structure is modified only after every field has passed validation. A partially valid packet can therefore never partially overwrite the shared dashboard state.
static telemetry_parse_result_t parse_telemetry_json(
    const char *json_text,
    telemetry_packet_t *output_packet
)
{
    if (json_text == NULL || output_packet == NULL) {
        return TELEMETRY_PARSE_INVALID_ARGUMENT;
    }

    // Require the complete input string to contain exactly one JSON value. This rejects trailing non-whitespace data after the JSON object.
    const char *parse_end = NULL;

    cJSON *root = cJSON_ParseWithOpts(
        json_text,
        &parse_end,
        true
    );

    if (root == NULL) {
        return TELEMETRY_PARSE_JSON_SYNTAX_ERROR;
    }

    telemetry_parse_result_t result = TELEMETRY_PARSE_OK;

    // Parse into a temporary local structure. output_packet remains unchanged if any later validation fails.
    telemetry_packet_t parsed_packet = {0};

    if (!cJSON_IsObject(root)) {
        result = TELEMETRY_PARSE_ROOT_NOT_OBJECT;
        goto cleanup;
    }

    // Retrieve every required protocol field by its exact case-sensitive name.
    const cJSON *protocol_version_item =
        cJSON_GetObjectItemCaseSensitive(root, "protocol_version");

    const cJSON *source_item =
        cJSON_GetObjectItemCaseSensitive(root, "source");

    const cJSON *sequence_item =
        cJSON_GetObjectItemCaseSensitive(root, "sequence");

    const cJSON *armed_item =
        cJSON_GetObjectItemCaseSensitive(root, "armed");

    const cJSON *mode_item =
        cJSON_GetObjectItemCaseSensitive(root, "mode");

    const cJSON *vbat_item =
        cJSON_GetObjectItemCaseSensitive(root, "vbat");

    const cJSON *motors_item =
        cJSON_GetObjectItemCaseSensitive(root, "motors");

    // Every field in protocol version 1 is mandatory.
    if (protocol_version_item == NULL ||
        source_item == NULL ||
        sequence_item == NULL ||
        armed_item == NULL ||
        mode_item == NULL ||
        vbat_item == NULL ||
        motors_item == NULL) {
        result = TELEMETRY_PARSE_MISSING_FIELD;
        goto cleanup;
    }

    // Validate JSON types before reading values. JSON booleans must be actual true/false values, not strings or numbers.
    if (!cJSON_IsNumber(protocol_version_item) ||
        !cJSON_IsString(source_item) ||
        !cJSON_IsNumber(sequence_item) ||
        !cJSON_IsBool(armed_item) ||
        !cJSON_IsString(mode_item) ||
        !cJSON_IsNumber(vbat_item) ||
        !cJSON_IsArray(motors_item)) {
        result = TELEMETRY_PARSE_WRONG_TYPE;
        goto cleanup;
    }

    // Protocol version must be an exact integer equal to the supported version.
    double protocol_version_value =
        protocol_version_item->valuedouble;

    if (protocol_version_value < 0.0 ||
        protocol_version_value > (double)UINT32_MAX ||
        (double)(uint32_t)protocol_version_value != protocol_version_value ||
        (uint32_t)protocol_version_value != TELEMETRY_PROTOCOL_VERSION) {
        result = TELEMETRY_PARSE_UNSUPPORTED_VERSION;
        goto cleanup;
    }

    // Source must identify the sender as the flight controller. The H2 test board will later send this source name while emulating the real flight controller.
    if (source_item->valuestring == NULL ||
        strcmp(
            source_item->valuestring,
            TELEMETRY_EXPECTED_SOURCE
        ) != 0) {
        result = TELEMETRY_PARSE_INVALID_SOURCE;
        goto cleanup;
    }

    // Sequence must be a non-negative uint32 integer. Duplicate and backwards-sequence detection will be applied in the packet-acceptance layer after parsing.
    double sequence_value = sequence_item->valuedouble;

    if (sequence_value < 0.0 ||
        sequence_value > (double)UINT32_MAX ||
        (double)(uint32_t)sequence_value != sequence_value) {
        result = TELEMETRY_PARSE_INVALID_SEQUENCE;
        goto cleanup;
    }

    // Validate mode length and allowed characters before copying it.
    if (mode_item->valuestring == NULL ||
        !telemetry_mode_is_valid(mode_item->valuestring)) {
        result = TELEMETRY_PARSE_INVALID_MODE;
        goto cleanup;
    }

    //Battery voltage is validated against broad electrical sanity limits. These are not low-battery or critical-battery thresholds.
    double vbat_value = vbat_item->valuedouble;

    if (vbat_value < TELEMETRY_VBAT_MIN_VOLTS ||
        vbat_value > TELEMETRY_VBAT_MAX_VOLTS) {
        result = TELEMETRY_PARSE_INVALID_VBAT;
        goto cleanup;
    }

    // Exactly four motor values are required for the current quadcopter telemetry protocol.
    if (cJSON_GetArraySize(motors_item) != 4) {
        result = TELEMETRY_PARSE_INVALID_MOTORS;
        goto cleanup;
    }

    for (int motor_index = 0; motor_index < 4; motor_index++) {
        const cJSON *motor_item =
            cJSON_GetArrayItem(motors_item, motor_index);

        if (!cJSON_IsNumber(motor_item)) {
            result = TELEMETRY_PARSE_INVALID_MOTORS;
            goto cleanup;
        }

        double motor_value = motor_item->valuedouble;

        // Motor output must be an integer inside the configured monitoring range. Fractional PWM-style values are rejected.
        if (motor_value < TELEMETRY_MOTOR_MIN ||
            motor_value > TELEMETRY_MOTOR_MAX ||
            (double)(int)motor_value != motor_value) {
            result = TELEMETRY_PARSE_INVALID_MOTORS;
            goto cleanup;
        }

        parsed_packet.motors[motor_index] = (int)motor_value;
    }

    // All validations succeeded. Populate the temporary packet.
    parsed_packet.protocol_version =
        (uint32_t)protocol_version_value;

    parsed_packet.sequence =
        (uint32_t)sequence_value;

    parsed_packet.armed =
        cJSON_IsTrue(armed_item);

    strncpy(
        parsed_packet.mode,
        mode_item->valuestring,
        sizeof(parsed_packet.mode) - 1
    );

    parsed_packet.mode[sizeof(parsed_packet.mode) - 1] = '\0';

    parsed_packet.vbat =
        (float)vbat_value;

    // Commit the complete validated packet to the caller.
    *output_packet = parsed_packet;

cleanup:
    // cJSON owns the complete parse tree. Deleting the root releases every child node allocated during parsing.
    cJSON_Delete(root);

    return result;
}

// Receives newline-delimited telemetry packets from UART. Each complete line is parsed and validated as one protocol packet. Invalid input never modifies the last known valid telemetry state.
static void telemetry_uart_task(void *arg)
{
    // This task currently receives no startup argument.
    (void)arg;

    uint8_t rx_byte = 0;

    // Buffer for one complete newline-terminated telemetry packet.
    char line_buffer[UART_LINE_BUFFER_SIZE] = {0};

    size_t line_pos = 0;

    ESP_LOGI(TAG, "UART telemetry task started.");

    ESP_LOGI(
        TAG,
        "Waiting for telemetry lines on UART%d at %d baud.",
        TELEMETRY_UART_PORT,
        TELEMETRY_UART_BAUD_RATE
    );

    while (1) {
        // Read one byte at a time so newline framing can be processed deterministically.
        int bytes_read = uart_read_bytes(
            TELEMETRY_UART_PORT,
            &rx_byte,
            1,
            pdMS_TO_TICKS(100)
        );

        if (bytes_read <= 0) {
            continue;
        }

        if (rx_byte == '\n') {
            // Ignore empty lines.
            if (line_pos == 0) {
                continue;
            }

            // Terminate the accumulated byte sequence as a C string.
            line_buffer[line_pos] = '\0';

            ESP_LOGI(
                TAG,
                "UART line received: %s",
                line_buffer
            );

            telemetry_packet_t parsed_packet = {0};

            telemetry_parse_result_t parse_result =
                parse_telemetry_json(
                    line_buffer,
                    &parsed_packet
                );

            if (parse_result == TELEMETRY_PARSE_OK) {
                // The complete packet passed syntax, type, and value validation. It is now safe to update shared state.
                latest_telemetry.armed =
                    parsed_packet.armed;

                strncpy(
                    latest_telemetry.mode,
                    parsed_packet.mode,
                    sizeof(latest_telemetry.mode) - 1
                );

                latest_telemetry.mode[
                    sizeof(latest_telemetry.mode) - 1
                ] = '\0';

                latest_telemetry.vbat =
                    parsed_packet.vbat;

                // Store the protocol sequence in the legacy counter field. This preserves the current HTTP/dashboard interface while the protocol migrates from counter to sequence terminology.
                latest_telemetry.counter =
                    parsed_packet.sequence;

                for (int motor_index = 0;
                     motor_index < 4;
                     motor_index++) {
                    latest_telemetry.motors[motor_index] =
                        parsed_packet.motors[motor_index];
                }

                // Update freshness only after the entire packet is accepted.
                latest_telemetry.last_packet_ms =
                    (uint32_t)(
                        esp_timer_get_time() / 1000ULL
                    );

                latest_telemetry.valid_packets++;

                ESP_LOGI(
                    TAG,
                    "Accepted telemetry: "
                    "version=%lu sequence=%lu armed=%d "
                    "mode=%s vbat=%.2f valid=%lu",
                    (unsigned long)parsed_packet.protocol_version,
                    (unsigned long)parsed_packet.sequence,
                    parsed_packet.armed,
                    parsed_packet.mode,
                    parsed_packet.vbat,
                    (unsigned long)latest_telemetry.valid_packets
                );
            } else {
                // The line was complete but did not satisfy the protocol. Keep the previous valid telemetry values unchanged.
                latest_telemetry.invalid_packets++;

                ESP_LOGW(
                    TAG,
                    "Rejected telemetry: reason=%s "
                    "invalid=%lu line=%s",
                    telemetry_parse_result_to_string(parse_result),
                    (unsigned long)latest_telemetry.invalid_packets,
                    line_buffer
                );
            }

            // Reset the line accumulator for the next packet.
            line_pos = 0;
            line_buffer[0] = '\0';
        }
        else if (rx_byte == '\r') {
            // Ignore carriage returns so both LF and CRLF senders work.
            continue;
        }
        else {
            if (line_pos < UART_LINE_BUFFER_SIZE - 1) {
                line_buffer[line_pos++] = (char)rx_byte;
            } else {
                // The packet exceeded the configured maximum line length. 
                // Drop the current buffer and count it as invalid.
                // The remaining bytes are ignored until a newline arrives.
                // A dedicated discard-until-newline state will be added in the next framing-hardening step.
                latest_telemetry.invalid_packets++;

                ESP_LOGW(
                    TAG,
                    "UART telemetry line exceeded %d bytes. "
                    "Dropping buffer. invalid=%lu",
                    UART_LINE_BUFFER_SIZE - 1,
                    (unsigned long)latest_telemetry.invalid_packets
                );

                line_pos = 0;
                line_buffer[0] = '\0';
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
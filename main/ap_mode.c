#include <esp_http_server.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <unistd.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char* TAG = "webserver";

extern char wifi_ssid[32];
extern char wifi_pass[64];

static esp_err_t home_page_handler(httpd_req_t* req) {
    // Buffer to hold the dynamically generated HTML content
    char* buf = malloc(2048);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for homepage content");
        return ESP_FAIL;
    }

    // Generate the HTML content with the current values of wifi_ssid and
    // wifi_pass
    snprintf(
        buf, 2048,
        "<!DOCTYPE html>\n"
        "<html lang=\"vi\">\n"
        "<head>\n"
        "    <title>IoT Gateway Setting</title>\n"
        "    <style>\n"
        "        body { font-family: sans-serif; }\n"
        "        .container { max-width: 600px; margin: 0 auto; padding: 20px; "
        "border: 1px solid #ccc; border-radius: 5px; }\n"
        "        input[type=\"text\"], select { width: 100%%; padding: 10px; "
        "border: 1px solid #ccc; border-radius: 3px; box-sizing: border-box; "
        "}\n"
        "        button[type=\"submit\"] { background-color: green; color: "
        "#fff; border: none; padding: 10px 20px; border-radius: 5px; cursor: "
        "pointer; }\n"
        "        button[type=\"submit\"]:hover { background-color: red; }\n"
        "    </style>\n"
        "</head>\n"
        "<body>\n"
        "    <form class=\"container\" action=\"/config\" method=\"get\">\n"
        "        <h1>IoT Gateway Setting</h1>\n"
        "        <table>\n"
        "            <tr><td>Wifi SSID</td><td><input type=\"text\" "
        "name=\"ssid\" value=\"%s\"></td></tr>\n"
        "            <tr><td>Wifi Password</td><td><input type=\"text\" "
        "name=\"password\" value=\"%s\"></td></tr>\n"
        "        </table>\n"
        "        <div class=\"button\"><button "
        "type=\"submit\">Submit</button></div>\n"
        "    </form>\n"
        "</body>\n"
        "</html>",
        wifi_ssid, wifi_pass);

    // Send the generated HTML content as the response
    esp_err_t ret = httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);  // Free the allocated memory for the HTML content
    return ret;
}

static const httpd_uri_t homepage = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = home_page_handler,
    .user_ctx = NULL  // Set to NULL as we will dynamically generate the HTML
                      // content in the handler
};

static esp_err_t config_wifi_handler(httpd_req_t* req) {
    char query[1024];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get query string");
        return ESP_FAIL;
    }

    char ssid[128], password[128];
    if (httpd_query_key_value(query, "ssid", ssid, sizeof(ssid)) != ESP_OK) {
        ESP_LOGE(TAG, "SSID not found in query");
        return ESP_FAIL;
    }

    if (httpd_query_key_value(query, "password", password, sizeof(password)) !=
        ESP_OK) {
        ESP_LOGE(TAG, "Password not found in query");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "SSID: %s", ssid);
    ESP_LOGI(TAG, "Password: %s", password);

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write
    err = nvs_set_str(my_handle, "ssid", ssid);
    if (err != ESP_OK) return err;
    err = nvs_set_str(my_handle, "password", password);
    if (err != ESP_OK) return err;

    // Commit written value.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);

    httpd_resp_send(req, "Reset Gateway to continue using",
                    strlen("Reset Gateway to continue using"));

    return ESP_OK;
}

static const httpd_uri_t config_wifi = {.uri = "/config",
                                        .method = HTTP_GET,
                                        .handler = config_wifi_handler,
                                        .user_ctx = NULL};

esp_err_t http_404_error_handler(httpd_req_t* req, httpd_err_code_t err) {
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.lru_purge_enable = true;
    config.stack_size = 8192;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &homepage);
        httpd_register_uri_handler(server, &config_wifi);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                     void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*)arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
#ifndef AP_MODE_H
#define AP_MODE_H

#include <esp_err.h>
#include <esp_http_server.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t http_404_error_handler(httpd_req_t* req, httpd_err_code_t err);
httpd_handle_t start_webserver(void);
esp_err_t stop_webserver(httpd_handle_t server);
void disconnect_handler(void* arg, esp_event_base_t event_base,
                        int32_t event_id, void* event_data);
void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                     void* event_data);

#ifdef __cplusplus
}
#endif

#endif  // AP_MODE_H
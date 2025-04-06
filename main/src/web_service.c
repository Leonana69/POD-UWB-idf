#include "nvs_flash.h"
#include "nvs.h"
#include "web_service.h"
#include "estimator.h"

static uint8_t get_id() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return 0;
    }

    uint8_t id = 0;
    err = nvs_get_u8(nvs_handle, "id", &id);
    if (err != ESP_OK) {
        id = 1;
        nvs_set_u8(nvs_handle, "id", id);
        nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return id;
}

static void set_id(uint8_t id) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }

    nvs_set_u8(nvs_handle, "id", id);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

/* Root */
esp_err_t root_get_handler(httpd_req_t *req) {
    const char* resp_str = "Athena UWB";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_uri_t root = {
    .uri      = "/",      // URI to handle
    .method   = HTTP_GET,
    .handler  = root_get_handler,
    .user_ctx = NULL
};

/* State */
esp_err_t state_get_handler(httpd_req_t *req) {
    state_t state;
    estimatorKalmanGetState(&state);
    char resp_str[256];
    snprintf(resp_str, sizeof(resp_str), "id: %d, x: %f, y: %f, z: %f",
            get_id(),  // Get the ID from NVS
            state.position.x, state.position.y, state.position.z);
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_uri_t state = {
    .uri      = "/state", // URI to handle
    .method   = HTTP_GET,
    .handler  = state_get_handler,
    .user_ctx = NULL
};

/* Set ID */
esp_err_t set_id_post_handler(httpd_req_t *req) {
    char buf[10];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0'; // Null-terminate the string

    uint8_t id = atoi(buf);
    if (id > 0 && id < 255) {
        set_id(id);
        const char* resp_str = "ID set successfully";
        httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    } else {
        const char* resp_str = "Invalid ID";
        httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

httpd_uri_t set_id_uri = {
    .uri      = "/set_id", // URI to handle
    .method   = HTTP_POST,
    .handler  = set_id_post_handler,
    .user_ctx = NULL
};

// Start web server
httpd_handle_t webServiceInit() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &state);
        httpd_register_uri_handler(server, &set_id_uri);
    }

    return server;
}
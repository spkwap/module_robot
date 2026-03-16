#include "robot.h"
#include "webpage.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>

#define ROBOT_WIFI_SSID    "Robo_Siec"
#define ROBOT_WIFI_PASS    ""

static const char *TAG = "NETWORK";

void network_init_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ROBOT_WIFI_SSID,
            .ssid_len = strlen(ROBOT_WIFI_SSID),
            .channel = 1,
            .password = ROBOT_WIFI_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK
        },
    };
    if (strlen(ROBOT_WIFI_PASS) == 0) wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Siec WiFi utworzona: %s", ROBOT_WIFI_SSID);
}

static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html"); 
    httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN); 
    return ESP_OK;
}

static esp_err_t stat_handler(httpd_req_t *req) {
    char buf[256]; 
    snprintf(buf, sizeof(buf), "{\"d\":\"%s\",\"o\":%d,\"m\":%d,\"f\":[%d,%d,%d,%d,%d]}", 
             g_dist, g_obs?1:0, g_mot?1:0, virtual_flags[0], virtual_flags[1], virtual_flags[2], virtual_flags[3], virtual_flags[4]);
    httpd_resp_set_type(req, "application/json"); 
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN); 
    return ESP_OK;
}

static esp_err_t api_rules_handler(httpd_req_t *req) {
    char buf[1024]; int ret = req->content_len; if (ret >= sizeof(buf)) return ESP_FAIL;
    if (ret > 0) { int recv_len = httpd_req_recv(req, buf, ret); if (recv_len <= 0) return ESP_FAIL; buf[recv_len] = '\0'; } else buf[0] = '\0';
    
    for(int i = 0; i < MAX_RULES; i++) system_rules[i].active = false;
    
    int idx = 0; char *rule_tok = strtok(buf, ";");
    while(rule_tok != NULL && idx < MAX_RULES) {
        int in1, op1, th1, in2, op2, th2, logL, ao, av, eo, ev, dur;
        if(sscanf(rule_tok, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &in1, &op1, &th1, &in2, &op2, &th2, &logL, &ao, &av, &eo, &ev, &dur) == 12) {
            system_rules[idx].active = true;
            system_rules[idx].in_type = (input_type_t)in1; system_rules[idx].op = (operator_t)op1; system_rules[idx].threshold = th1;
            system_rules[idx].in_type_2 = (input_type_t)in2; system_rules[idx].op_2 = (operator_t)op2; system_rules[idx].threshold_2 = th2;
            system_rules[idx].logic_link = logL;
            system_rules[idx].act_out = (output_type_t)ao; system_rules[idx].act_val = av;
            system_rules[idx].els_out = (output_type_t)eo; system_rules[idx].els_val = ev;
            system_rules[idx].duration_ms = dur; 
            idx++;
        }
        rule_tok = strtok(NULL, ";");
    }
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN); 
    return ESP_OK;
}

void webserver_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); config.server_port = 80;
    httpd_handle_t server = NULL; 
    if (httpd_start(&server, &config) != ESP_OK) return;
    
    httpd_uri_t uri_root  = { .uri = "/", .method = HTTP_GET, .handler = root_handler };
    httpd_uri_t uri_stat  = { .uri = "/api/stat", .method = HTTP_GET, .handler = stat_handler }; 
    httpd_uri_t uri_rules = { .uri = "/api/rules", .method = HTTP_POST, .handler = api_rules_handler };
    
    httpd_register_uri_handler(server, &uri_root); 
    httpd_register_uri_handler(server, &uri_stat); 
    httpd_register_uri_handler(server, &uri_rules);
}
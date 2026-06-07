#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "robot.h"
#include "vl53l0x.h"

#define LED_PIN 2

char g_dist[16] = "--";
char g_i2c_status[128] = "Skanowanie...";
bool g_obs = false;
bool g_mot = false;
int virtual_flags[5] = {0, 0, 0, 0, 0}; 
rule_t system_rules[MAX_RULES];

volatile bool is_system_ready = false;

bool evaluate_condition(input_type_t in_type, operator_t op, int threshold, uint16_t dist, int tcrt) {
    if (in_type == IN_NONE) return false;
    int val_to_check = 0;
    if (in_type == IN_DISTANCE) val_to_check = dist;
    else if (in_type == IN_TCRT) val_to_check = tcrt;
    else if (in_type >= IN_FLAG_1 && in_type <= IN_FLAG_5) {
        val_to_check = virtual_flags[in_type - IN_FLAG_1];
    }
    
    if(op == OP_LESS) return (val_to_check < threshold);
    if(op == OP_GREATER) return (val_to_check > threshold);
    if(op == OP_EQUAL) return (val_to_check == threshold);
    return false;
}

static void sensor_task(void *pv) {
    vl53l0x_t *sensor = vl53l0x_config_with_bus(i2c_bus, -1, 0x29, 1);
    if (sensor && !vl53l0x_init(sensor)) {
        vl53l0x_setMeasurementTimingBudget(sensor, 50000); 
        vl53l0x_startContinuous(sensor, 200);
    }

    uint64_t active_timer_end = 0;
    int locked_ml = 0; int locked_mr = 0; bool timer_is_active = false;

    while (1) {
        uint16_t current_dist = 9999;
        if (sensor) {
            uint16_t range = vl53l0x_readRangeContinuousMillimeters(sensor);
            if (vl53l0x_timeoutOccurred(sensor) || range >= 8190) { strcpy(g_dist, "OOR"); current_dist = 9999; } 
            else { snprintf(g_dist, sizeof(g_dist), "%d", range); current_dist = range; }
        }
        
        bool pin_state = read_tcrt_sensor();
        g_obs = !pin_state; 
        int current_tcrt = g_obs ? 1 : 0; 

        int target_ml = 0; int target_mr = 0; 
        uint64_t now_ms = esp_timer_get_time() / 1000ULL; 

        if (timer_is_active && now_ms < active_timer_end) {
            target_ml = locked_ml; target_mr = locked_mr;
        } 
        else {
            timer_is_active = false; 
            int pending_duration = 0; 

            for(int i = 0; i < MAX_RULES; i++) {
                if(!system_rules[i].active) continue;

                bool cond_1 = evaluate_condition(system_rules[i].in_type, system_rules[i].op, system_rules[i].threshold, current_dist, current_tcrt);
                
                bool final_condition = cond_1;
                if (system_rules[i].logic_link == 1) { 
                    bool cond_2 = evaluate_condition(system_rules[i].in_type_2, system_rules[i].op_2, system_rules[i].threshold_2, current_dist, current_tcrt);
                    final_condition = (cond_1 && cond_2);
                } else if (system_rules[i].logic_link == 2) { 
                    bool cond_2 = evaluate_condition(system_rules[i].in_type_2, system_rules[i].op_2, system_rules[i].threshold_2, current_dist, current_tcrt);
                    final_condition = (cond_1 || cond_2);
                }

                output_type_t out_type = final_condition ? system_rules[i].act_out : system_rules[i].els_out;
                int out_val            = final_condition ? system_rules[i].act_val : system_rules[i].els_val;

                if(out_type == OUT_MOTOR_LEFT) { target_ml = out_val; }
                else if(out_type == OUT_MOTOR_RIGHT) { target_mr = out_val; }
                else if(out_type == OUT_MOTORS_BOTH) { target_ml = out_val; target_mr = out_val; }
                else if(out_type >= OUT_FLAG_1 && out_type <= OUT_FLAG_5) {
                    virtual_flags[out_type - OUT_FLAG_1] = out_val; 
                }

                if (final_condition && (out_type == OUT_MOTOR_LEFT || out_type == OUT_MOTOR_RIGHT || out_type == OUT_MOTORS_BOTH)) {
                    pending_duration = system_rules[i].duration_ms; 
                }
            }

            if (pending_duration > 0) {
                active_timer_end = now_ms + pending_duration;
                locked_ml = target_ml; locked_mr = target_mr;
                timer_is_active = true;
            }
        }

        motors_set_speed(target_ml, target_mr); 
        g_mot = ((target_ml != 0) || (target_mr != 0));

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

static void scan_i2c_devices() {
    int devices_found = 0;
    char temp_buf[16];
    
    strcpy(g_i2c_status, "Wykryte I2C: ");
    
    for (uint16_t i = 1; i < 127; i++) {
        esp_err_t ret = i2c_master_probe(i2c_bus, i, 10);

        if (ret == ESP_OK) {
            snprintf(temp_buf, sizeof(temp_buf), "0x%02X ", i);
            strcat(g_i2c_status, temp_buf);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        strcpy(g_i2c_status, "Brak podlaczonych modulow I2C!");
    }
}

static void led_status_task(void *pv) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    int led_state = 0;
    
    while (!is_system_ready) {
        gpio_set_level(LED_PIN, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(150)); 
    }
    
    gpio_set_level(LED_PIN, 1); 
    
    vTaskDelete(NULL); 
}

void app_main(void) {
    xTaskCreate(led_status_task, "led_task", 2048, NULL, 1, NULL);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { 
        nvs_flash_erase(); nvs_flash_init(); 
    }
    
    hardware_init();
    scan_i2c_devices();
    network_init_ap();
    webserver_start();
    
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);

    is_system_ready = true;
}
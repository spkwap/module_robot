#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vl53l0x.h"
#include "pca9685.h"
#include "pcf8575.h"

#define I2C_PORT    I2C_NUM_0
#define SDA_PIN     21
#define SCL_PIN     22
#define XSHUT_PIN   -1 

#define PIN_LED     7
#define PIN_BUTTON  8

#define SPEED_FWD    60
#define SPEED_BACK  -60
#define SPEED_STOP   0
#define DIST_LIMIT   200 

static const char *TAG = "ROBOT_FULL_SYSTEM";

void app_main()
{
    vl53l0x_t *sensor = vl53l0x_config(I2C_PORT, SCL_PIN, SDA_PIN, XSHUT_PIN, 0x29, 0);
    
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Błąd alokacji I2C");
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "Start PCF8575...");
    if (pcf8575_init(I2C_PORT) == ESP_OK) {
        pcf8575_set_pin(I2C_PORT, PIN_LED, 1); vTaskDelay(200/portTICK_PERIOD_MS);
        pcf8575_set_pin(I2C_PORT, PIN_LED, 0);
        
        pcf8575_set_pin(I2C_PORT, PIN_BUTTON, 1);
    } else {
        ESP_LOGE(TAG, "Błąd PCF8575");
    }

    ESP_LOGI(TAG, "Start PCA9685...");
    pca9685_init(I2C_PORT);
    pca9685_set_pwm_freq(I2C_PORT, 50);
    pca9685_set_servo_speed(I2C_PORT, 0, 0);

    ESP_LOGI(TAG, "Start VL53L0X...");
    int retry = 0;
    while (vl53l0x_init(sensor) != NULL) {
        retry++;
        ESP_LOGW(TAG, "Próba czujnika %d...", retry);
        vTaskDelay(pdMS_TO_TICKS(500));
        if (retry > 5) break;
    }
    
    vl53l0x_setTimeout(sensor, 1000);
    vl53l0x_startContinuous(sensor, 0);
    

    bool pause_mode = false;

    while (1) {
        if (pcf8575_get_pin(I2C_PORT, PIN_BUTTON) == 0) {
            
            vTaskDelay(pdMS_TO_TICKS(50));
            if (pcf8575_get_pin(I2C_PORT, PIN_BUTTON) == 0) {
                
                pause_mode = !pause_mode;
                ESP_LOGW(TAG, "Przycisk wciśnięty, tryb cofania: %d", pause_mode);
                
                while(pcf8575_get_pin(I2C_PORT, PIN_BUTTON) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }
        pcf8575_set_pin(I2C_PORT, PIN_BUTTON, 1);

        
        if (pause_mode) {
            pca9685_set_servo_speed(I2C_PORT, 0, SPEED_STOP);
            pcf8575_set_pin(I2C_PORT, PIN_LED, 1);
        } 
        else {
            pcf8575_set_pin(I2C_PORT, PIN_LED, 0);
            
            uint16_t distance = vl53l0x_readRangeContinuousMillimeters(sensor);
            
            if (vl53l0x_timeoutOccurred(sensor) || distance > 8000 || distance == 0) {
                distance = 9999; 
            }

            if (distance < DIST_LIMIT) {
                ESP_LOGI(TAG, "Przeszkoda (%d mm)", distance);
                pca9685_set_servo_speed(I2C_PORT, 0, SPEED_BACK);
            } else {
                pca9685_set_servo_speed(I2C_PORT, 0, SPEED_FWD);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
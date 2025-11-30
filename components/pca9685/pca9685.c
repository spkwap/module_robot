#include "pca9685.h"
#include "esp_log.h"
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "PCA9685";

#define MODE1 0x00
#define PRE_SCALE 0xFE
#define LED0_ON_L 0x06

static esp_err_t write_byte(i2c_port_t port, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t read_byte(i2c_port_t port, uint8_t reg) {
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return data;
}

esp_err_t pca9685_init(i2c_port_t port) {
    esp_err_t err = write_byte(port, MODE1, 0x00);
    return err;
}

esp_err_t pca9685_set_pwm_freq(i2c_port_t port, uint16_t freq_hz) {
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= (float)freq_hz;
    prescaleval -= 1.0;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5);

    uint8_t oldmode = read_byte(port, MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;
    write_byte(port, MODE1, newmode);
    write_byte(port, PRE_SCALE, prescale);
    write_byte(port, MODE1, oldmode);
    vTaskDelay(pdMS_TO_TICKS(5));
    write_byte(port, MODE1, oldmode | 0xA0);
    
    ESP_LOGI(TAG, "Częstotliwość ustawiona na %d Hz (Prescaler: %d)", freq_hz, prescale);
    return ESP_OK;
}

esp_err_t pca9685_set_pwm(i2c_port_t port, uint8_t channel, uint16_t on, uint16_t off) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LED0_ON_L + 4 * channel, true);
    i2c_master_write_byte(cmd, on & 0xFF, true);
    i2c_master_write_byte(cmd, on >> 8, true);
    i2c_master_write_byte(cmd, off & 0xFF, true);
    i2c_master_write_byte(cmd, off >> 8, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void pca9685_set_servo_speed(i2c_port_t port, uint8_t channel, int speed_percent) {
    
    const int STOP_PULSE = 307;
    const int MAX_OFFSET = 102;

    if (speed_percent > 100) speed_percent = 100;
    if (speed_percent < -100) speed_percent = -100;

    int pwm_val = STOP_PULSE + (speed_percent * MAX_OFFSET / 100);
    
    pca9685_set_pwm(port, channel, 0, pwm_val);
}
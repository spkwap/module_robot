#ifndef PCA9685_H
#define PCA9685_H

#include "driver/i2c.h"

#define PCA9685_ADDR 0x40

esp_err_t pca9685_init(i2c_port_t port);
esp_err_t pca9685_set_pwm_freq(i2c_port_t port, uint16_t freq_hz);
esp_err_t pca9685_set_pwm(i2c_port_t port, uint8_t channel, uint16_t on, uint16_t off);
void pca9685_set_servo_speed(i2c_port_t port, uint8_t channel, int speed_percent);

#endif
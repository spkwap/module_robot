#ifndef PCF8575_H
#define PCF8575_H

#include "driver/i2c.h"

#define PCF8575_ADDR 0x21 

esp_err_t pcf8575_init(i2c_port_t port);

esp_err_t pcf8575_write(i2c_port_t port, uint16_t data);

uint16_t pcf8575_read(i2c_port_t port);

void pcf8575_set_pin(i2c_port_t port, uint8_t pin, uint8_t val);
uint8_t pcf8575_get_pin(i2c_port_t port, uint8_t pin);
void pcf8575_toggle_pin(i2c_port_t port, uint8_t pin);

#endif
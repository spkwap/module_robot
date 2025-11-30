#include "pcf8575.h"
#include "esp_log.h"

static uint16_t _port_state = 0xFFFF; 

esp_err_t pcf8575_init(i2c_port_t port) {
    _port_state = 0xFFFF;
    return pcf8575_write(port, _port_state);
}

esp_err_t pcf8575_write(i2c_port_t port, uint16_t data) {
    _port_state = data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8575_ADDR << 1) | I2C_MASTER_WRITE, true);
    
    i2c_master_write_byte(cmd, data & 0xFF, true); 
    i2c_master_write_byte(cmd, (data >> 8) & 0xFF, true);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

uint16_t pcf8575_read(i2c_port_t port) {
    uint8_t low = 0, high = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8575_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &low, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &high, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return (high << 8) | low;
}

void pcf8575_set_pin(i2c_port_t port, uint8_t pin, uint8_t val) {
    if (val) _port_state |= (1 << pin);
    else     _port_state &= ~(1 << pin);
    pcf8575_write(port, _port_state);
}

uint8_t pcf8575_get_pin(i2c_port_t port, uint8_t pin) {
    uint16_t state = pcf8575_read(port);
    return (state >> pin) & 1;
}

void pcf8575_toggle_pin(i2c_port_t port, uint8_t pin) {
    _port_state ^= (1 << pin);
    pcf8575_write(port, _port_state);
}
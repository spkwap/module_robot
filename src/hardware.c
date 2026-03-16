#include "robot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define I2C_PORT          0
#define I2C_SDA_IO        21
#define I2C_SCL_IO        22
#define PCF8575_ADDRESS   0x20
#define PCA9685_ADDRESS   0x40
#define TCRT5000_PIN      0 
#define MOTOR_CHANNEL_1   12 
#define MOTOR_CHANNEL_2   15 

i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t pcf_dev = NULL;
static i2c_master_dev_handle_t pca_dev = NULL;

static void pca9685_write_byte(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data}; i2c_master_transmit(pca_dev, buf, 2, pdMS_TO_TICKS(100));
}
static uint8_t pca9685_read_byte(uint8_t reg) {
    uint8_t val = 0; i2c_master_transmit_receive(pca_dev, &reg, 1, &val, 1, pdMS_TO_TICKS(100)); return val;
}
static void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t buf[5] = { (uint8_t)(0x06 + 4 * channel), (uint8_t)(on & 0xFF), (uint8_t)(on >> 8), (uint8_t)(off & 0xFF), (uint8_t)(off >> 8) };
    i2c_master_transmit(pca_dev, buf, 5, pdMS_TO_TICKS(100));
}

void hardware_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT, .sda_io_num = I2C_SDA_IO, .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
    
    i2c_device_config_t pcf_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = PCF8575_ADDRESS, .scl_speed_hz = 100000, };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &pcf_cfg, &pcf_dev));
    
    i2c_device_config_t pca_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = PCA9685_ADDRESS, .scl_speed_hz = 100000, };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &pca_cfg, &pca_dev));

    pca9685_write_byte(0x00, 0x00); vTaskDelay(pdMS_TO_TICKS(10));
    float prescale = roundf(25000000.0f / (4096.0f * 50)) - 1.0f;
    uint8_t oldmode = pca9685_read_byte(0x00);
    pca9685_write_byte(0x00, (oldmode & 0x7F) | 0x10); 
    pca9685_write_byte(0xFE, (uint8_t)prescale); 
    pca9685_write_byte(0x00, oldmode); 
    vTaskDelay(pdMS_TO_TICKS(5)); 
    pca9685_write_byte(0x00, oldmode | 0xA0);
    motors_stop();
}

bool read_tcrt_sensor(void) {
    uint8_t buf[2] = {0xFF, 0xFF}; 
    if (i2c_master_receive(pcf_dev, buf, 2, pdMS_TO_TICKS(100)) != ESP_OK) {
        return true; 
    }
    return (((uint16_t)buf[1] << 8) | buf[0]) >> TCRT5000_PIN & 1;
}

void motors_set_speed(int target_ml, int target_mr) {
    const int STOP_PULSE = 307; const int MAX_OFFSET = 102;
    
    int spd_l = target_ml;
    if(spd_l > 100) spd_l = 100;
    if(spd_l < -100) spd_l = -100;
    pca9685_set_pwm(MOTOR_CHANNEL_1, 0, STOP_PULSE + (spd_l * MAX_OFFSET / 100));

    int spd_r = -target_mr;
    if(spd_r > 100) spd_r = 100;
    if(spd_r < -100) spd_r = -100;
    pca9685_set_pwm(MOTOR_CHANNEL_2, 0, STOP_PULSE + (spd_r * MAX_OFFSET / 100));
}

void motors_stop(void) {
    motors_set_speed(0, 0);
}
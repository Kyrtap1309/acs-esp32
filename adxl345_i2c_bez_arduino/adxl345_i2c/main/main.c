#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_SDA_IO        15
#define I2C_MASTER_SCL_IO        16
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_TIMEOUT_MS    1000

#define ADXL345_ADDR             0x53
#define ADXL345_REG_DEVID        0x00
#define ADXL345_REG_POWER_CTL    0x2D
#define ADXL345_REG_DATA_FORMAT  0x31
#define ADXL345_REG_DATAX0       0x32

static const char *TAG = "ADXL345";

esp_err_t adxl345_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t adxl345_read_register(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void adxl345_init() {
    uint8_t devid = 0;
    adxl345_read_register(ADXL345_REG_DEVID, &devid, 1);
    ESP_LOGI(TAG, "Device ID: 0x%02X", devid);
    adxl345_write_register(ADXL345_REG_POWER_CTL, 0x08); // wake up
    adxl345_write_register(ADXL345_REG_DATA_FORMAT, 0x01); // ±4g, 
}

void adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    adxl345_read_register(ADXL345_REG_DATAX0, data, 6);
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
}

void app_main(void) {
    i2c_master_init();
    adxl345_init();

    while (1) {
        int16_t x, y, z;
        adxl345_read_xyz(&x, &y, &z);
        ESP_LOGI(TAG, "X: %d, Y: %d, Z: %d", x, y, z);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

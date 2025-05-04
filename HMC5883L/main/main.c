#include <stdio.h>
#include <math.h> 
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   15
#define I2C_MASTER_SCL_IO   16
#define I2C_MASTER_FREQ_HZ  50000  

#define QMC5883L_ADDR           0x0D
#define QMC5883L_REG_DATA_X_LSB 0x00
#define QMC5883L_REG_DATA_X_MSB 0x01
#define QMC5883L_REG_DATA_Y_LSB 0x02
#define QMC5883L_REG_DATA_Y_MSB 0x03
#define QMC5883L_REG_DATA_Z_LSB 0x04
#define QMC5883L_REG_DATA_Z_MSB 0x05
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_CONTROL1   0x09
#define QMC5883L_REG_CONTROL2   0x0A
#define QMC5883L_REG_PERIOD     0x0B

#define QMC5883L_MODE_STANDBY   0x00
#define QMC5883L_MODE_CONTINUOUS 0x01
#define QMC5883L_ODR_10HZ       0x00
#define QMC5883L_ODR_50HZ       0x04
#define QMC5883L_ODR_100HZ      0x08
#define QMC5883L_ODR_200HZ      0x0C
#define QMC5883L_RNG_2G         0x00
#define QMC5883L_RNG_8G         0x10
#define QMC5883L_OSR_512        0x00
#define QMC5883L_OSR_256        0x40
#define QMC5883L_OSR_128        0x80
#define QMC5883L_OSR_64         0xC0

static const char *TAG = "QMC5883L";

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883l_data_t;

static esp_err_t qmc5883l_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t qmc5883l_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t qmc5883l_init() {
    esp_err_t err;
    
    err = qmc5883l_write_byte(QMC5883L_REG_CONTROL2, 0x80);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset QMC5883L, error: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
    
    // tryb ciagly, 200Hz, 8G, oversampling 512
    uint8_t ctrl1_val = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_200HZ | 
                       QMC5883L_RNG_8G | QMC5883L_OSR_512;
    err = qmc5883l_write_byte(QMC5883L_REG_CONTROL1, ctrl1_val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure QMC5883L, error: %s", esp_err_to_name(err));
        return err;
    }
    
    // ustawienie okresu SET/RESET
    err = qmc5883l_write_byte(QMC5883L_REG_PERIOD, 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set period, error: %s", esp_err_to_name(err));
        return err;
    }
    
    return ESP_OK;
}

static bool qmc5883l_data_ready() {
    uint8_t status;
    if (qmc5883l_read_reg(QMC5883L_REG_STATUS, &status, 1) != ESP_OK) {
        return false;
    }
    return (status & 0x01) != 0; 
}

static esp_err_t qmc5883l_read_data(qmc5883l_data_t *data) {
    uint8_t buf[6]; 
    
    
    esp_err_t err = qmc5883l_read_reg(QMC5883L_REG_DATA_X_LSB, buf, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data, error: %s", esp_err_to_name(err));
        return err;
    }
    
    data->x = (int16_t)(buf[1] << 8 | buf[0]);
    data->y = (int16_t)(buf[3] << 8 | buf[2]);
    data->z = (int16_t)(buf[5] << 8 | buf[4]);
    
    return ESP_OK;
}

static void i2c_master_init() {
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

static void i2c_scan() {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    for (uint8_t i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at address 0x%02x", i);
        }
    }
    
    ESP_LOGI(TAG, "Scan completed");
}

void app_main(void) {
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized");
    
    i2c_scan();
    
    if (qmc5883l_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init QMC5883L");
        return;
    }
    ESP_LOGI(TAG, "QMC5883L initialized successfully");

    qmc5883l_data_t data;
    
    while (1) {
        uint8_t retries = 0;
        while (!qmc5883l_data_ready() && retries < 10) {
            vTaskDelay(pdMS_TO_TICKS(10));
            retries++;
        }
        
        if (qmc5883l_read_data(&data) == ESP_OK) {
            ESP_LOGI(TAG, "Magnetic field: X=%d Y=%d Z=%d", data.x, data.y, data.z);
            
            double heading = atan2((double)data.y, (double)data.x) * 180.0 / M_PI;
            if (heading < 0) {
                heading += 360.0;
            }
            ESP_LOGI(TAG, "Heading: %.1f°", heading);
        } else {
            ESP_LOGW(TAG, "Read error");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
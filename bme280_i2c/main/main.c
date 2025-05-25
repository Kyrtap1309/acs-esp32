#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           16      // GPIO pin for SCL
#define I2C_MASTER_SDA_IO           15      // GPIO pin for SDA
#define I2C_MASTER_NUM              0       // I2C port number
#define I2C_MASTER_FREQ_HZ          100000  // I2C frequency (100 KHz)
#define I2C_MASTER_TIMEOUT_MS       1000

#define BME280_SENSOR_ADDR          0x76    // BME280 sensor address (can be 0x76 or 0x77)
#define BME280_REG_ID               0xD0    // ID register
#define BME280_REG_RESET            0xE0    // Reset register
#define BME280_REG_CTRL_HUM         0xF2    // Humidity control register
#define BME280_REG_STATUS           0xF3    // Status register
#define BME280_REG_CTRL_MEAS        0xF4    // Measurement control register
#define BME280_REG_CONFIG           0xF5    // Configuration register
#define BME280_REG_PRESS            0xF7    // Pressure data register
#define BME280_REG_TEMP             0xFA    // Temperature data register
#define BME280_REG_HUM              0xFD    // Humidity data register

// Calibration registers
#define BME280_REG_DIG_T1           0x88
#define BME280_REG_DIG_T2           0x8A
#define BME280_REG_DIG_T3           0x8C
#define BME280_REG_DIG_P1           0x8E
#define BME280_REG_DIG_H1           0xA1
#define BME280_REG_DIG_H2           0xE1

// Calibration parameters
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_data_t;

static const char *TAG = "BME280";
static bme280_calib_data_t calib_data;
static int32_t t_fine;

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

static esp_err_t bme280_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    
    return i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, 
                                     write_buf, sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bme280_read_reg(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR,
                                                &reg_addr, 1, data, len,
                                                I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err;
}

static esp_err_t bme280_read_calibration_data(void)
{
    uint8_t data[24];
    esp_err_t err;

    err = bme280_read_reg(BME280_REG_DIG_T1, data, 24);
    if (err != ESP_OK) {
        return err;
    }

    calib_data.dig_T1 = (data[1] << 8) | data[0];
    calib_data.dig_T2 = (data[3] << 8) | data[2];
    calib_data.dig_T3 = (data[5] << 8) | data[4];

    calib_data.dig_P1 = (data[7] << 8) | data[6];
    calib_data.dig_P2 = (data[9] << 8) | data[8];
    calib_data.dig_P3 = (data[11] << 8) | data[10];
    calib_data.dig_P4 = (data[13] << 8) | data[12];
    calib_data.dig_P5 = (data[15] << 8) | data[14];
    calib_data.dig_P6 = (data[17] << 8) | data[16];
    calib_data.dig_P7 = (data[19] << 8) | data[18];
    calib_data.dig_P8 = (data[21] << 8) | data[20];
    calib_data.dig_P9 = (data[23] << 8) | data[22];

    err = bme280_read_reg(BME280_REG_DIG_H1, &calib_data.dig_H1, 1);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t h_data[7];
    err = bme280_read_reg(BME280_REG_DIG_H2, h_data, 7);
    if (err != ESP_OK) {
        return err;
    }

    calib_data.dig_H2 = (h_data[1] << 8) | h_data[0];
    calib_data.dig_H3 = h_data[2];
    calib_data.dig_H4 = (h_data[3] << 4) | (h_data[4] & 0x0F);
    calib_data.dig_H5 = (h_data[5] << 4) | (h_data[4] >> 4);
    calib_data.dig_H6 = h_data[6];

    return ESP_OK;
}

static esp_err_t bme280_init(void)
{
    uint8_t chip_id;
    esp_err_t err;
    
    err = bme280_read_reg(BME280_REG_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading sensor ID: %d", err);
        return err;
    }
    
    if (chip_id != 0x60) {
        ESP_LOGE(TAG, "Invalid sensor ID: 0x%02x, expected 0x60", chip_id);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "BME280 sensor ID: 0x%02x", chip_id);
    
    err = bme280_write_reg(BME280_REG_RESET, 0xB6);
    if (err != ESP_OK) {
        return err;
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS); // Time for reset
    
    err = bme280_read_calibration_data();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading calibration data: %d", err);
        return err;
    }
    
    err = bme280_write_reg(BME280_REG_CTRL_HUM, 0x01);  // x1 oversampling
    if (err != ESP_OK) {
        return err;
    }
    
    err = bme280_write_reg(BME280_REG_CTRL_MEAS, 0x27);  // temp x1, press x1, normal mode
    if (err != ESP_OK) {
        return err;
    }
    
    err = bme280_write_reg(BME280_REG_CONFIG, 0x00);  // no filter, standby time 0.5ms
    if (err != ESP_OK) {
        return err;
    }
    
    return ESP_OK;
}

static float bme280_compensate_temperature(int32_t adc_temp)
{
    int32_t var1, var2;
    
    var1 = ((((adc_temp >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_temp >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_temp >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    float temperature = (t_fine * 5 + 128) >> 8;
    
    return temperature / 100.0f;
}

static float bme280_compensate_pressure(int32_t adc_press)
{
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // Avoid division by zero
    }
    
    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    
    return (float)p / 256.0f;
}

static float bme280_compensate_humidity(int32_t adc_hum)
{
    int32_t v_x1_u32r;
    
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_hum << 14) - (((int32_t)calib_data.dig_H4) << 20) - (((int32_t)calib_data.dig_H5) * v_x1_u32r)) +
               ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) * (((v_x1_u32r *
               ((int32_t)calib_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
               ((int32_t)calib_data.dig_H2) + 8192) >> 14));
    
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    
    return (float)(v_x1_u32r >> 12) / 1024.0f;
}

static esp_err_t bme280_read_measurements(float *temperature, float *pressure, float *humidity)
{
    uint8_t data[8];
    esp_err_t err;
    
    err = bme280_read_reg(BME280_REG_PRESS, data, 8);
    if (err != ESP_OK) {
        return err;
    }
    
    // Convert measurement data
    int32_t adc_press = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_temp = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);
    int32_t adc_hum = ((uint32_t)data[6] << 8) | data[7];
    
    *temperature = bme280_compensate_temperature(adc_temp);
    *pressure = bme280_compensate_pressure(adc_press) / 100.0f; // hPa
    *humidity = bme280_compensate_humidity(adc_hum);
    
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;
    
    err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization error: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    err = bme280_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 initialization error: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "BME280 initialized successfully");
    
    float temperature, pressure, humidity;
    
    while (1) {
        err = bme280_read_measurements(&temperature, &pressure, &humidity);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%", temperature, pressure, humidity);
        } else {
            ESP_LOGE(TAG, "Error reading measurements: %s", esp_err_to_name(err));
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Read every 2 seconds
    }
}
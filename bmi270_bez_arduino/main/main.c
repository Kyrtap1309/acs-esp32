#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK 12
#define PIN_NUM_CS 10

#define BMI270_CHIP_ID_REG 0x00
#define BMI270_CHIP_ID     0x24
#define BMI270_ACC_CONF    0x40
#define BMI270_ACC_RANGE   0x41
#define BMI270_PWR_CTRL    0x7D
#define BMI270_PWR_CONF    0x7C
#define BMI270_DATA_START  0x0C

spi_device_handle_t spi;

void spi_write_byte(uint8_t reg_addr, uint8_t data) {
    spi_transaction_t t = {
        .length = 16, // adres 8 bitow i dane 8 bitow
        .tx_buffer = (uint8_t[]){reg_addr & 0x7F, data}, // Trzeba wyzerowac najmniej istotny bit do zapisu
    };
    spi_device_polling_transmit(spi, &t);
}

uint8_t spi_read_byte(uint8_t reg_addr) {
    uint8_t rx_data = 0;
    spi_transaction_t t = {
        .length = 16, // adres 8 bitow i dane 8 bitow
        .tx_buffer = (uint8_t[]){reg_addr | 0x80}, // Trzeba ustawic najmniej istotny bit do odczytu
        .rx_buffer = &rx_data,
    };
    spi_device_polling_transmit(spi, &t);
    return rx_data;
}

void bmi270_init() {
    // Oczyt dummy by przestawic na SPI
    uint8_t dummy = spi_read_byte(BMI270_CHIP_ID_REG);
    // Odczyt ID aby sprawdzic czy dziala komunikacja SPI 
    uint8_t chip_id = spi_read_byte(BMI270_CHIP_ID_REG);
    if (chip_id != BMI270_CHIP_ID) {
        printf("Blad, nie wykryto chipa, CHIP ID: 0x%02X\n", chip_id);
        return;
    }
    printf("BMI270 wykryty, CHIP ID: 0x%02X\n", chip_id);

    // Wylacza oszczedzanie energii
    spi_write_byte(BMI270_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Wlacz akcelerometr i zyroskop
    spi_write_byte(BMI270_PWR_CTRL, 0x0E);

    // Konfiguracja akceleroemetru
    spi_write_byte(BMI270_ACC_CONF, 0xA8);

    // Zakres akcelerometru
    spi_write_byte(BMI270_ACC_RANGE, 0x01);

    printf("BMI270 skonfigurowany.\n");
}

void bmi270_read_accel() {
    uint8_t data[6];
    spi_transaction_t t = {
        .length = 8 * sizeof(data),
        .tx_buffer = (uint8_t[]){BMI270_DATA_START | 0x80},
        .rx_buffer = data,
    };
    spi_device_polling_transmit(spi, &t);

    int16_t ax = (int16_t)((data[1] << 8) | data[0]);
    int16_t ay = (int16_t)((data[3] << 8) | data[2]);
    int16_t az = (int16_t)((data[5] << 8) | data[4]);

    printf("Dane akcelerometru: AX=%d, AY=%d, AZ=%d\n", ax, ay, az);
}

void app_main() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000, // 5 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    // Wlacz SPI
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // Dodaj BMI270
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Inicjalizacja bmi270
    bmi270_init();

    while (1) {
        //bmi270_read_accel();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Definicje pinów SPI
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   14  // Chip Select dla BMP384

// Rejestry BMP384
#define BMP384_REG_CHIP_ID      0x00  // Rejestr ID
#define BMP384_REG_STATUS       0x03  // Rejestr statusu
#define BMP384_REG_CTRL_MEAS    0x1B  // Rejestr kontrolny pomiaru
#define BMP384_REG_CONFIG       0x1C  // Rejestr konfiguracji
#define BMP384_REG_PRESS_MSB    0x04  // Dane ciśnienia (MSB)
#define BMP384_REG_TEMP_MSB     0x07  // Dane temperatury (MSB)
#define BMP384_CHIP_ID          0x50  // Oczekiwany Chip ID

// Struktura kalibracji
typedef struct {
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
    float t_lin;  // Zmienna pomocnicza dla temperatury skompensowanej
} bmp384_calib_data_t;

// Rozmiary odczytu
#define BMP384_PRESS_DATA_LEN   3
#define BMP384_TEMP_DATA_LEN    3

// Struktura do przechowywania uchwytu SPI
static spi_device_handle_t bmp384_spi;

// Funkcja do inicjalizacji SPI
void spi_init() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  // 1 MHz
        .mode = 0,                          // Tryb SPI (CPOL = 0, CPHA = 0)
        .spics_io_num = PIN_NUM_CS,         // Pin CS
        .queue_size = 7,                    // Kolejka transakcji
    };

    // Inicjalizacja magistrali SPI
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // Dodanie urządzenia BMP384
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &bmp384_spi));
}

// Funkcja do odczytu rejestrów BMP384
uint8_t bmp384_read_register(uint8_t reg_addr) {
    uint8_t tx_data[2] = {reg_addr | 0x80, 0x00};  // Ustawienie MSB na 1 (odczyt)
    uint8_t rx_data[2];

    spi_transaction_t t = {
        .length = 16,          // 8 bitów adresu + 8 bitów danych
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    ESP_ERROR_CHECK(spi_device_transmit(bmp384_spi, &t));
    return rx_data[1];  // Dane z rejestru
}

// Funkcja do odczytu wielu bajtów
void bmp384_read_registers(uint8_t reg_addr, uint8_t *data, size_t len) {
    uint8_t tx_data = reg_addr | 0x80;  // Ustawienie MSB na 1 (odczyt)
    spi_transaction_t t = {
        .length = 8 * (len + 1),
        .tx_buffer = &tx_data,
        .rx_buffer = data,
        .rxlength = 8 * (len + 1)
    };

    ESP_ERROR_CHECK(spi_device_transmit(bmp384_spi, &t));
    memmove(data, data + 1, len);  // Przesunięcie danych, aby pozbyć się pierwszego bajtu
}

// Funkcja do zapisu rejestru BMP384
void bmp384_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_data[2] = {reg_addr & 0x7F, value};  // Ustawienie MSB na 0 (zapis)

    spi_transaction_t t = {
        .length = 16,          // 8 bitów adresu + 8 bitów danych
        .tx_buffer = tx_data,
    };

    ESP_ERROR_CHECK(spi_device_transmit(bmp384_spi, &t));
}

// Funkcja inicjalizacji BMP384
void bmp384_init() {
    // Sprawdzenie ID urządzenia
    uint8_t chip_id = bmp384_read_register(BMP384_REG_CHIP_ID);
    if (chip_id != BMP384_CHIP_ID) {
        printf("Nieprawidłowy CHIP ID: 0x%02X\n", chip_id);
        return;
    }

    printf("BMP384 CHIP ID: 0x%02X\n", chip_id);

    // Ustawienia konfiguracji czujnika
    bmp384_write_register(BMP384_REG_CTRL_MEAS, 0x33);  // Normal mode, Temp i Press ON
    bmp384_write_register(BMP384_REG_CONFIG, 0x10);     // Filtr IIR i standby time
}

// Funkcja do odczytu współczynników kalibracji
void bmp384_read_calibration(bmp384_calib_data_t *calib_data) {
    uint8_t calib_regs[21];  // Rozmiar tabeli kalibracji (zgodnie z dokumentacją)

    // Odczyt danych kalibracyjnych z rejestrów czujnika
    bmp384_read_registers(0x31, calib_regs, 21);

    // Przetwarzanie współczynników kalibracji zgodnie z dokumentacją
    calib_data->par_t1 = (float)(((uint16_t)calib_regs[1] << 8) | calib_regs[0]) / (1 << -8);
    calib_data->par_t2 = (float)(((int16_t)calib_regs[3] << 8) | calib_regs[2]) / (1 << 30);
    calib_data->par_t3 = (float)calib_regs[4] / (1 << 48);

    calib_data->par_p1 = (float)(((int16_t)calib_regs[6] << 8) | calib_regs[5]) / (1 << 20);
    calib_data->par_p2 = (float)(((int16_t)calib_regs[8] << 8) | calib_regs[7]) / (1 << 29);
    calib_data->par_p3 = (float)calib_regs[9] / (1 << 32);
    calib_data->par_p4 = (float)(((int16_t)calib_regs[11] << 8) | calib_regs[10]) / (1 << 37);
    calib_data->par_p5 = (float)(((int16_t)calib_regs[13] << 8) | calib_regs[12]) / (1 << -3);
    calib_data->par_p6 = (float)calib_regs[14] / (1 << 6);
    calib_data->par_p7 = (float)calib_regs[15] / (1 << 8);
    calib_data->par_p8 = (float)calib_regs[16] / (1 << 15);
    calib_data->par_p9 = (float)(((int16_t)calib_regs[18] << 8) | calib_regs[17]) / (1 << 48);
    calib_data->par_p10 = (float)calib_regs[19] / (1 << 48);
    calib_data->par_p11 = (float)calib_regs[20] / (1 << 65);
}

// Funkcja do odczytu danych RAW (surowych)
void bmp384_read_raw(int32_t *temp_raw, int32_t *press_raw) {
    uint8_t temp_data[BMP384_TEMP_DATA_LEN];
    uint8_t press_data[BMP384_PRESS_DATA_LEN];

    // Odczyt temperatury
    bmp384_read_registers(BMP384_REG_TEMP_MSB, temp_data, BMP384_TEMP_DATA_LEN);
    *temp_raw = ((int32_t)temp_data[0] << 16) | ((int32_t)temp_data[1] << 8) | temp_data[2];

    // Odczyt ciśnienia
    bmp384_read_registers(BMP384_REG_PRESS_MSB, press_data, BMP384_PRESS_DATA_LEN);
    *press_raw = ((int32_t)press_data[0] << 16) | ((int32_t)press_data[1] << 8) | press_data[2];
}

// Funkcja do kompensacji temperatury
float bmp384_compensate_temperature(int32_t uncomp_temp, bmp384_calib_data_t *calib_data) {
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
    partial_data2 = partial_data1 * calib_data->par_t2;

    calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;

    return calib_data->t_lin;
}

// Funkcja do kompensacji ciśnienia
float bmp384_compensate_pressure(int32_t uncomp_press, bmp384_calib_data_t *calib_data) {
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_out1;
    float partial_out2;

    partial_data1 = calib_data->t_lin * calib_data->t_lin;
    partial_data2 = partial_data1 * calib_data->t_lin;
    partial_data3 = calib_data->par_p6 * partial_data2 + calib_data->par_p5 * partial_data1 +
                    calib_data->par_p4 * calib_data->t_lin + calib_data->par_p3;

    partial_out1 = calib_data->par_p2 * uncomp_press + calib_data->par_p1;
    partial_out2 = calib_data->par_p9 * uncomp_press * uncomp_press + calib_data->par_p8 * uncomp_press;

    return partial_out1 + partial_out2 + partial_data3;
}

// Główna funkcja aplikacji
void app_main() {
    spi_init();
    bmp384_init();

    bmp384_calib_data_t calib_data;

    // Odczyt współczynników kalibracji
    bmp384_read_calibration(&calib_data);

    while (1) {
        int32_t temp_raw = 0, press_raw = 0;
        float temperature = 0.0, pressure = 0.0;

        // Odczyt danych surowych
        bmp384_read_raw(&temp_raw, &press_raw);

        // Kompensacja temperatury
        temperature = bmp384_compensate_temperature(temp_raw, &calib_data);

        // Kompensacja ciśnienia
        pressure = bmp384_compensate_pressure(press_raw, &calib_data);

        // Wyświetlenie wyników
        printf("Temperature: %.2f °C, Pressure: %.2f Pa\n", temperature, pressure);

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 sekunda opóźnienia
    }
}

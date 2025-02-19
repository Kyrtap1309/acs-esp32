#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define SPI_MOSI_PIN 11  // Pin MOSI
#define SPI_MISO_PIN 13  // Pin MISO
#define SPI_CLK_PIN  12  // Pin zegara SPI
#define SPI_CS_PIN   14   // Pin Chip Select (CS)

//Adresy Rejestrow (Nazwy zgodne z datasheetem)
#define PWR_CTRL_REG 0x1B
#define OSR_REG 0x1C
#define CONFIG_REG 0x1F
#define ODR_REG 0x1D


#define NVM_PAR_T1 0x31
#define NVM_PAR_T2 0x33
#define NVM_PAR_T3 0x35
#define NVM_PAR_P1 0x36
#define NVM_PAR_P2 0x38
#define NVM_PAR_P3 0x3A
#define NVM_PAR_P4 0x3B
#define NVM_PAR_P5 0x3C
#define NVM_PAR_P6 0x3E
#define NVM_PAR_P7 0x40
#define NVM_PAR_P8 0x41
#define NVM_PAR_P9 0x42
#define NVM_PAR_P10 0x44
#define NVM_PAR_P11 0x45

static const char *TAG = "SPI_Transaction";

spi_device_handle_t spi;

typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    float t_lin;
} bmp384_calib_data_t;

// Wpisywanie bajtu do rejestru
esp_err_t bmp384_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx_buffer[2];  // Dane do wysłania: 7-bitowy adres + dummy
    tx_buffer[0] = reg & 0x7F;  // 7-bitowy adres z opuszczonym MSB
    tx_buffer[1] = value;         // dane

    uint8_t rx_buffer[2];        // Bufor na dane odbierane

    spi_transaction_t trans = {
        .length = 2 * 8,          // Długość danych do wysłania (3 bajty = 24 bity)
        .rxlength = 2 * 8,        // Długość danych do odbioru (3 bajty = 24 bity)
        .tx_buffer = tx_buffer,   // Bufor danych do wysłania
        .rx_buffer = rx_buffer    // Bufor danych do odbioru
    };

    esp_err_t ret = spi_device_transmit(spi, &trans); // Przeprowadzenie transakcji

    return ret;
}

//Wpisywanie wiele bajtow do rejestru (i kolejnych)
esp_err_t bmp384_write_register_many_bytes(uint8_t reg, uint8_t* data, size_t length) {
    uint8_t *tx_buffer = (uint8_t *)malloc(length + 1);
    uint8_t *rx_buffer = (uint8_t *) malloc(length + 1);

    if (!tx_buffer || !rx_buffer) {
        ESP_LOGE(TAG, "Memory allocation failed!");
        free(tx_buffer);
        free(rx_buffer);
        return ESP_ERR_NO_MEM;
    }


    tx_buffer[0] = reg & 0x7F;
    
    memcpy(&tx_buffer[1], data, length);

    spi_transaction_t trans = {
        .length = (length + 1) * 8,
        .rxlength = (length + 1) * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };

    ESP_LOGI(TAG, "Transmisja pliku konfiguracyjnego");
    esp_err_t ret = spi_device_transmit(spi, &trans);
    ESP_LOGI(TAG, "Zakonczono transmisje pliku konfiguracyjnego");

    ESP_LOGI(TAG, "Zwalnianie buforow...");
    free(tx_buffer);
    free(rx_buffer);
    ESP_LOGI(TAG, "Bufory zwolnione");

    return ret;
}



esp_err_t bmp384_read_register(uint8_t reg, uint8_t *data) {

    uint8_t tx_buffer[3];  // Dane do wysłania: 7-bitowy adres + dummy
    tx_buffer[0] = reg | 0x80;  // 7-bitowy adres z ustawionym MSB
    tx_buffer[1] = 0x00;         // Dummy bajt
    tx_buffer[2] = 0x00;         // dane

    uint8_t rx_buffer[3];        // Bufor na dane odbierane

    spi_transaction_t trans = {
        .length = 3 * 8,          // Długość danych do wysłania (3 bajty = 24 bity)
        .rxlength = 3 * 8,        // Długość danych do odbioru (3 bajty = 24 bity)
        .tx_buffer = tx_buffer,   // Bufor danych do wysłania
        .rx_buffer = rx_buffer    // Bufor danych do odbioru
    };

    esp_err_t ret = spi_device_transmit(spi, &trans); // Przeprowadzenie transakcji
    
    *data = rx_buffer[2];

    return ret;
}

esp_err_t bmp384_read_register_many_bytes(uint8_t reg, uint8_t* data, size_t length) {
    
    uint8_t tx_buffer[length + 2];
    uint8_t rx_buffer[length + 2];

    tx_buffer[0] = reg | 0x80;
    tx_buffer[1] = 0x00;

    for (size_t i = 2; i < length + 2; i++){
        tx_buffer[i] = 0x00;
    }


    spi_transaction_t trans = {
        .length = (length + 2) * 8,
        .rxlength = (length + 2) * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };

    esp_err_t ret = spi_device_transmit(spi, &trans);

    if (ret == ESP_OK) {
        for (size_t i = 0; i < length; i++) {
            data[i] = rx_buffer[i + 2];
        }
    }
    return ret;

}

void bmp384_read_calibration(bmp384_calib_data_t *calib, uint32_t uncomp_temp) {
    uint8_t data[2];

    bmp384_read_register_many_bytes(NVM_PAR_T1, data, 2);
    calib->par_t1 = (uint16_t)((data[1] << 8) | data[0]) * 256.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_T2, data, 2);
    calib->par_t2 = (uint16_t)((data[1] << 8) | data[0]) / 1073741824.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_T3, data, 1);
    calib->par_t3 = (int8_t)data[0] / ((float)(1ULL << 48));
    
    bmp384_read_register_many_bytes(NVM_PAR_P1, data, 2);
    calib->par_p1 = ((int16_t)((data[1] << 8) | data[0]) - 16384) / 1048576.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P2, data, 2);
    calib->par_p2 = ((int16_t)((data[1] << 8) | data[0]) - 16384) / 536870912.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P3, data, 1);
    calib->par_p3 = (int8_t)data[0] / 4294967296.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P4, data, 1);
    calib->par_p4 = (int8_t)data[0] / 137438953472.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P5, data, 2);
    calib->par_p5 = ((uint16_t)((data[1] << 8) | data[0])) * 8.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P6, data, 2);
    calib->par_p6 = ((uint16_t)((data[1] << 8) | data[0])) / 64.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P7, data, 1);
    calib->par_p7 = (int8_t)data[0] / 256.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P8, data, 1);
    calib->par_p8 = (int8_t)data[0] / 32768.0;
    
    bmp384_read_register_many_bytes(NVM_PAR_P9, data, 2);
    calib->par_p9 = ((int16_t)((data[1] << 8) | data[0])) / ((float)(1ULL << 48));
    
    bmp384_read_register_many_bytes(NVM_PAR_P10, data, 1);
    calib->par_p10 = (int8_t)data[0] / ((float)(1ULL << 48));
    
    bmp384_read_register_many_bytes(NVM_PAR_P11, data, 1);
    calib->par_p11 = (int8_t)data[0] / ((float)(1ULL << 48));

    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - calib->par_t1);
    partial_data2 = (float)(partial_data1 * calib->par_t2);

    calib->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib->par_t3;
}

float bmp384_compensate_pressure(uint32_t uncomp_press, bmp384_calib_data_t *calib) {
    float partial_data1 = calib->par_p6 * calib->t_lin;
    float partial_data2 = calib->par_p7 * (calib->t_lin * calib->t_lin);
    float partial_data3 = calib->par_p8 * (calib->t_lin * calib->t_lin * calib->t_lin);
    float partial_out1 = calib->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib->par_p2 * calib->t_lin;
    partial_data2 = calib->par_p3 * (calib->t_lin * calib->t_lin);
    partial_data3 = calib->par_p4 * (calib->t_lin * calib->t_lin * calib->t_lin);
    float partial_out2 = (float)uncomp_press * (calib->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib->par_p9 + calib->par_p10 * calib->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 * ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press)*calib->par_p11;

    return partial_out1 + partial_out2 + partial_data4;
}

void init_spi() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,  // Nieużywane
        .quadhd_io_num = -1,  // Nieużywane
        .max_transfer_sz = 100 // Maksymalny rozmiar transferu (w bajtach)
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 400000,         // Prędkość zegara SPI: 1 MHz
        .mode = 0,                         // Tryb SPI: SPI_MODE0
        .spics_io_num = SPI_CS_PIN,        // Pin CS
        .queue_size = 1                    // Kolejka transakcji
    };

    // Inicjalizacja magistrali SPI
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // Dodanie urządzenia podrzędnego
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));


    
}

esp_err_t bmp384_init() {
    
    ESP_LOGI(TAG, "Inicjalizacja bmp384");

    // Odczyt cisnienia i temperatury w trybie Normal
    bmp384_write_register(PWR_CTRL_REG, 0x33);

    // Rozdzielczosc pomiaru cisnienia 21 bit/ 0.085 Pa
    // Rozdzielczosc temperatury 17 bit / 0.0025 C
    // Pressure oversampling x32 Temperature oversampling x2
    bmp384_write_register(OSR_REG, 0x0D);

    // Wsp filtra IIR
    bmp384_write_register(CONFIG_REG, 0x04);

    // Czestotliwosc odswiezania 50 Hz
    bmp384_write_register(ODR_REG, 0x02);

    return ESP_OK;
}


esp_err_t bmp384_check() {

    uint8_t chip_id;

    // Read chip ID
    if (bmp384_read_register(0x00, &chip_id) != ESP_OK || chip_id != 0x50) {
        ESP_LOGE(TAG, "BMP384 nie wykryty! Chip ID: 0x%02X", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMP384 wykryty. Chip ID: 0x%02X", chip_id);


    return ESP_OK;
}

esp_err_t bmp384_read_data()
{
    uint8_t raw_data[6];
    esp_err_t ret = bmp384_read_register_many_bytes(0x04, raw_data, 6);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read bmp384 data");
        return ret;
    }

    // Przyspieszenia
    uint32_t uncomp_press = (uint32_t)((raw_data[2] << 16) | (raw_data[1] << 8) | (raw_data[0]));
    uint32_t uncomp_temp = (uint32_t)((raw_data[6] << 16) | (raw_data[5] << 8) | (raw_data[4]));

    bmp384_calib_data_t calib_data;

    bmp384_read_calibration(&calib_data, uncomp_temp);
    
    ESP_LOGI(TAG, "Odczytano dane kalibracyjne poprawnie.");

    float comp_press = bmp384_compensate_pressure(uncomp_press, &calib_data);

    ESP_LOGI(TAG, "Cisnienie: = %.2f\n", comp_press);

    return ESP_OK;
    

}


void app_main() {
    
    init_spi();

    if (bmp384_init() != ESP_OK) {
        ESP_LOGE(TAG, "Fail inicjalizacji BMP384");
        return;
    }
    ESP_LOGI(TAG, "Inicjalizacja BMP384 przebiegla pomyslnie");

    if (bmp384_check() != ESP_OK) {
        ESP_LOGE(TAG, "Fail konfiguracji BMP384!");
        return;
    }
    ESP_LOGI(TAG, "Konfiguracja BMP384 przebiegla pomyslnie!");

    bmp384_read_data();

}

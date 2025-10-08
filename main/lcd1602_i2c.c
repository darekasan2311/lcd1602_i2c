#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_FREQ_HZ      100000
#define LCD_I2C_ADDRESS         0x27 // I2C address of LCD
#define SCD4x_I2C_ADDRESS       0x62

// LCD Commands
#define LCD_CLEARDISPLAY        0x01
#define LCD_CURSORRETURN        0x02
#define LCD_INPUTSET            0x04
#define LCD_SET_DDRAM_ADR       0x80

// SCD41 Commands
#define SCD41_CMD_START_PERIODIC_MEASUREMENT 0x21B1
#define SCD41_CMD_READ_MEASUREMENT 0xEC05
#define SCD41_CMD_STOP_PERIODIC_MEASUREMENT 0x3F86
#define SCD41_CMD_GET_SERIAL_NUMBER 0x3682
#define SCD41_CMD_PERFORM_SELF_TEST 0x3639

// Timing delays for SCD41
#define SCD41_MEASUREMENT_DELAY_MS 5000  // 5 seconds between measurements
#define SCD41_CMD_DELAY_MS 1             // 1ms command execution time

// Rows
#define LCD_ROW1                0x00
#define LCD_ROW2                0x40

// LCD buffer
#define LCD_BUFFERSIZE          64

// Flags for backlight control
#define PIN_BACKLIGHT           0x0C
#define PIN_NOBACKLIGHT         0x00

#define PIN_ENABLE              0x04     // Enable bit
#define PIN_RW                  0x02     // Read/Write bit
#define PIN_RS                  0x01     // Register select bit

static const char *TAG = "SCD41";

static i2c_master_dev_handle_t lcd_handle;
static i2c_master_dev_handle_t scd_handle;
static i2c_master_bus_handle_t bus_handle;

static uint8_t state_backlight = PIN_BACKLIGHT; // DEFAULT ON

static void add_i2c_dev(uint8_t address, i2c_master_dev_handle_t *dev_handle, const char *device_name);

void i2c_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    add_i2c_dev(LCD_I2C_ADDRESS, &lcd_handle, "LCD1602");
    add_i2c_dev(SCD4x_I2C_ADDRESS, &scd_handle, "SCD4x");
}

static void add_i2c_dev(uint8_t address, i2c_master_dev_handle_t *dev_handle, const char *device_name)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

        esp_err_t ret = i2c_master_probe(bus_handle, address, pdMS_TO_TICKS(250));
        if (ret == ESP_OK)
        {
            ESP_LOGI("I2C", "Found device at 0x%02X", address);
            esp_err_t add_ok = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
            if (add_ok != ESP_OK)
            {
                ESP_LOGE("I2C", "Adding device %s (0x%02X) failed: %s",
                        device_name, address, esp_err_to_name(ret));
            }
            else
            {
                ESP_LOGI("I2C", "Device %s (0x%02X) added successfully",
                        device_name, address);
            }
        } else {
            ESP_LOGE("I2C", "Device not found at 0x%02X", address);
            ESP_LOGE("I2C", "Abort adding device.");
        }
    
}

static void lcd_send_byte(uint8_t data)
{
    i2c_master_transmit(lcd_handle, &data, 1, -1);
}

static void lcd_pulse(uint8_t data)
{
    lcd_send_byte(data | PIN_ENABLE);  // E=1
    esp_rom_delay_us(1);               // >450 ns
    lcd_send_byte(data & ~PIN_ENABLE); // E=0
    esp_rom_delay_us(50);              // >37 us
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | state_backlight;
    if (mode)
        data |= PIN_RS; // RS=1 for data, 0 for command
    lcd_pulse(data);
}

static void lcd_send_cmd(uint8_t cmd)
{
    lcd_send_nibble(cmd & 0xF0, 0);        // high nibble
    lcd_send_nibble((cmd << 4) & 0xF0, 0); // low nibble
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_send_data(uint8_t val)
{
    lcd_send_nibble(val & 0xF0, 1);
    lcd_send_nibble((val << 4) & 0xF0, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t address;
    if (row == 0)
        address = LCD_ROW1 + col;
    else
        address = LCD_ROW2 + col;

    lcd_send_cmd(LCD_SET_DDRAM_ADR | address);
}

void lcd_init(void)
{
    // ---- LCD Init sequence ----
    vTaskDelay(pdMS_TO_TICKS(50)); // wait after power-on

    lcd_send_cmd(0x33); // Initialize
    lcd_send_cmd(0x32); // Set to 4-bit mode
    lcd_send_cmd(0x28); // 4-bit, 2 line, 5x8 font
    lcd_send_cmd(0x0C); // Display ON, cursor OFF, blink OFF
    lcd_send_cmd(0x06); // Entry mode: cursor moves right
    lcd_send_cmd(0x01); // Clear display
    vTaskDelay(pdMS_TO_TICKS(50));
}

void lcd_clear(void)
{
    lcd_send_cmd(LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void lcd_backlight_off(void)
{
    state_backlight = PIN_NOBACKLIGHT;
    lcd_send_cmd(state_backlight);
}

void lcd_backlight_on(void)
{
    state_backlight = PIN_BACKLIGHT;
    lcd_send_cmd(state_backlight);
}

void lcd_print(const char *data)
{
    while (*data)
    {
        lcd_send_data(*data);
        data++;
    }
}

void lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char buffer[LCD_BUFFERSIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    lcd_set_cursor(row, col);
    lcd_print(buffer);
}

// CRC-8 calculation (polynomial: 0x31, init: 0xFF)
static uint8_t calculate_crc(uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

// Send command to SCD41
esp_err_t scd41_send_command(uint16_t cmd) {
    uint8_t buf[2];
    buf[0] = (cmd >> 8) & 0xFF;  // MSB
    buf[1] = cmd & 0xFF;          // LSB

    esp_err_t ret = i2c_master_transmit(scd_handle, buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%04X: %s", cmd, esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(SCD41_CMD_DELAY_MS));
    return ESP_OK;
}

// Start periodic measurement
esp_err_t scd41_start_periodic_measurement(void) {
    esp_err_t ret = scd41_send_command(SCD41_CMD_START_PERIODIC_MEASUREMENT);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Periodic measurement started");
    }
    return ret;
}

// Stop periodic measurement
esp_err_t scd41_stop_periodic_measurement(void) {
    esp_err_t ret = scd41_send_command(SCD41_CMD_STOP_PERIODIC_MEASUREMENT);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));  // Wait for stop command to complete
        ESP_LOGI(TAG, "Periodic measurement stopped");
    }
    return ret;
}

// Read measurement data (CO2, Temperature, Humidity)
esp_err_t scd41_read_measurement(uint16_t *co2, float *temperature, float *humidity) {
    esp_err_t ret;
    uint8_t cmd_buf[2];
    uint8_t data[9];  // 3 words × 3 bytes (2 data + 1 CRC)

    // Send read measurement command
    cmd_buf[0] = (SCD41_CMD_READ_MEASUREMENT >> 8) & 0xFF;
    cmd_buf[1] = SCD41_CMD_READ_MEASUREMENT & 0xFF;

    ret = i2c_master_transmit(scd_handle, cmd_buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read command: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // Wait for data to be ready

    // Read 9 bytes (CO2, Temp, RH with CRCs)
    ret = i2c_master_receive(scd_handle, data, 9, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement: %s", esp_err_to_name(ret));
        return ret;
    }

    // Verify CRCs and extract data
    for (int i = 0; i < 3; i++) {
        uint8_t crc = calculate_crc(&data[i * 3], 2);
        if (crc != data[i * 3 + 2]) {
            ESP_LOGE(TAG, "CRC error at word %d", i);
            return ESP_ERR_INVALID_CRC;
        }
    }

    // Extract CO2 (ppm)
    *co2 = (data[0] << 8) | data[1];

    // Extract Temperature (°C) = -45 + 175 * (value / 65536)
    uint16_t temp_raw = (data[3] << 8) | data[4];
    *temperature = -45.0f + 175.0f * (temp_raw / 65536.0f);

    // Extract Humidity (%RH) = 100 * (value / 65536)
    uint16_t hum_raw = (data[6] << 8) | data[7];
    *humidity = 100.0f * (hum_raw / 65536.0f);

    return ESP_OK;
}

// Get serial number
esp_err_t scd41_get_serial_number(uint64_t *serial) {
    esp_err_t ret;
    uint8_t cmd_buf[2];
    uint8_t data[9];  // 3 words × 3 bytes

    cmd_buf[0] = (SCD41_CMD_GET_SERIAL_NUMBER >> 8) & 0xFF;
    cmd_buf[1] = SCD41_CMD_GET_SERIAL_NUMBER & 0xFF;

    ret = i2c_master_transmit(scd_handle, cmd_buf, 2, -1);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    ret = i2c_master_receive(scd_handle, data, 9, -1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Verify CRCs
    for (int i = 0; i < 3; i++) {
        uint8_t crc = calculate_crc(&data[i * 3], 2);
        if (crc != data[i * 3 + 2]) {
            return ESP_ERR_INVALID_CRC;
        }
    }

    // Combine into 48-bit serial number
    *serial = ((uint64_t)data[0] << 40) | ((uint64_t)data[1] << 32) |
              ((uint64_t)data[3] << 24) | ((uint64_t)data[4] << 16) |
              ((uint64_t)data[6] << 8) | data[7];

    return ESP_OK;
}

static void scd41_print(void) 
{
    uint16_t co2;
    float temperature, humidity;

    if (scd41_read_measurement(&co2, &temperature, &humidity) == ESP_OK) {
        ESP_LOGI(TAG, "CO2: %d ppm, Temperature: %.2f °C, Humidity: %.2f %%RH",
                    co2, temperature, humidity);
    } else {
        ESP_LOGW(TAG, "Failed to read measurement");
    }

    vTaskDelay(pdMS_TO_TICKS(SCD41_MEASUREMENT_DELAY_MS));
}

void app_main(void)
{
    uint64_t serial;

    i2c_init();
    lcd_init();

    vTaskDelay(pdMS_TO_TICKS(SCD41_MEASUREMENT_DELAY_MS));

        // Get serial number
    if (scd41_get_serial_number(&serial) == ESP_OK) {
        ESP_LOGI(TAG, "Serial Number: 0x%012llX", serial);
    }
    lcd_printf(0, 0, "Serial Number:");
    lcd_printf(1, 0, "0x%012llX", serial);

    // Start periodic measurements
    ESP_ERROR_CHECK(scd41_start_periodic_measurement());

    // Wait for first measurement (5 seconds)
    ESP_LOGI(TAG, "Waiting for first measurement...");
    vTaskDelay(pdMS_TO_TICKS(SCD41_MEASUREMENT_DELAY_MS));

    while(1) {
        scd41_print();
    }
}

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_FREQ_HZ 100000

#define PIN_BACKLIGHT (1 << 3) // some boards: P3/P4 swapped, adjust!
#define PIN_ENABLE (1 << 2)
#define PIN_RW (1 << 1)
#define PIN_RS (1 << 0)

i2c_master_dev_handle_t dev_handle;

static void lcd_send_byte(uint8_t data)
{
    i2c_master_transmit(dev_handle, &data, 1, -1);
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
    uint8_t data = (nibble & 0xF0) | PIN_BACKLIGHT;
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

void app_main(void)
{

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x27,
        .scl_speed_hz = 100000,
    };

    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle) != ESP_OK)
    {
        ESP_LOGE("I2C", "ADDING DEVICE FAILED");
    }
    else
    {
        ESP_LOGI("I2C", "DEVICE ADDED SUCCESSFULLY");
    };

    lcd_init();
    lcd_send_data('H');
    lcd_send_data('i');
    // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data_wr, DATA_LENGTH, -1));
}

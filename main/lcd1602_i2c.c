#include <stdio.h>
#include <stdarg.h>

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

void app_main(void)
{
    i2c_init();
    lcd_init();
    lcd_printf(0, 0, "Added dev on %x", LCD_I2C_ADDRESS);
    lcd_printf(1, 0, "Added dev on %x", SCD4x_I2C_ADDRESS);
}

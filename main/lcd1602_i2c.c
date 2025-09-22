#include <stdio.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_FREQ_HZ      100000
#define LCD_I2C_ADDRESS         0x27    // I2C address of LCD 

// LCD Commands
#define LCD_CLEARDISPLAY        0x01
#define LCD_CURSORRETURN        0x02
#define LCD_INPUTSET            0x04
// #define LCD_DISPLAYCONTROL      0x08
// #define LCD_CURSORSHIFT         0x10
// #define LCD_FUNCTIONSET         0x20
// #define LCD_SETCGRAMADDR        0x40
// #define LCD_SETDDRAMADDR        0x80

// // Flags for display entry mode
// #define LCD_ENTRYRIGHT          0x00
// #define LCD_ENTRYLEFT           0x02
// #define LCD_ENTRYSHIFTINCREMENT 0x01
// #define LCD_ENTRYSHIFTDECREMENT 0x00

// // Flags for display on/off control
// #define LCD_DISPLAYON           0x04
// #define LCD_DISPLAYOFF          0x00
// #define LCD_CURSORON            0x02
// #define LCD_CURSOROFF           0x00
// #define LCD_BLINKON             0x01
// #define LCD_BLINKOFF            0x00

// // Flags for function set
// #define LCD_8BITMODE            0x10
// #define LCD_4BITMODE            0x00
// #define LCD_2LINE               0x08
// #define LCD_1LINE               0x00
// #define LCD_5x10DOTS            0x04
// #define LCD_5x8DOTS             0x00

// Flags for backlight control
#define PIN_BACKLIGHT           0x0C
#define PIN_NOBACKLIGHT         0x00

#define PIN_ENABLE              0x04  // Enable bit
#define PIN_RW                  0x02  // Read/Write bit
#define PIN_RS                  0x01  // Register select bit

i2c_master_dev_handle_t dev_handle;
i2c_master_bus_handle_t bus_handle;

static uint8_t state_backlight = PIN_BACKLIGHT; // DEFAULT ON

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

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
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
    vTaskDelay(pdMS_TO_TICKS(2000));
    // lcd_clear();
    lcd_backlight_off();
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_backlight_on();
    // lcd_send_cmd(0x00);
    // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data_wr, DATA_LENGTH, -1));
}

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_FREQ_HZ 100000

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

    printf("Scanning I2C bus...\n");

    for (uint8_t addr = 1; addr < 127; addr++)
    {
        esp_err_t ret = i2c_master_probe(bus_handle, addr, pdMS_TO_TICKS(250));
        if (ret == ESP_OK)
        {
            printf("Found device at 0x%02X\n", addr);
        }
    }
}

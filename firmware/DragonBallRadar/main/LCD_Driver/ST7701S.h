#ifndef RGB_PANEL_ST7701S_H
#define RGB_PANEL_ST7701S_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"

#define SPI_METHOD 1
#define IOEXPANDER_METHOD 0

typedef struct{
    char method_select;
    //SPI config_t
    spi_device_handle_t spi_device;
    spi_bus_config_t spi_io_config_t;
    spi_device_interface_config_t st7701s_protocol_config_t;
}ST7701S;

typedef ST7701S * ST7701S_handle;

//Create new object
ST7701S_handle ST7701S_newObject(int SDA, int SCL, int CS, char channel_select, char method_select);

//Screen initialization
void ST7701S_screen_init(ST7701S_handle St7701S_handle, unsigned char type);

//Delete object
void ST7701S_delObject(ST7701S_handle St7701S_handle);

//SPI write instruction
void ST7701S_WriteCommand(ST7701S_handle St7701S_handle, uint8_t cmd);

//SPI write data
void ST7701S_WriteData(ST7701S_handle St7701S_handle, uint8_t data);

#endif

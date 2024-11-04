#include "ST7701S.h"

#define SPI_WriteComm(cmd) ST7701S_WriteCommand(St7701S_handle, cmd)
#define SPI_WriteData(data) ST7701S_WriteData(St7701S_handle, data)
#define Delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS)

void ioexpander_init(){};
void ioexpander_write_cmd(){};
void ioexpander_write_data(){};

/**
 * @brief Example Create an ST7701S object
 * @param SDA SDA pin
 * @param SCL SCL pin
 * @param CS  CS  pin
 * @param channel_select SPI channel selection
 * @param method_select  SPI_METHOD,IOEXPANDER_METHOD
 * @note
*/
ST7701S_handle ST7701S_newObject(int SDA, int SCL, int CS, char channel_select, char method_select)
{
    // if you use `malloc()`, please set 0 in the area to be assigned.
    ST7701S_handle st7701s_handle = heap_caps_calloc(1, sizeof(ST7701S), MALLOC_CAP_DEFAULT);
    st7701s_handle->method_select = method_select;
    
    if(method_select){
        st7701s_handle->spi_io_config_t.miso_io_num = -1;
        st7701s_handle->spi_io_config_t.mosi_io_num = SDA;
        st7701s_handle->spi_io_config_t.sclk_io_num = SCL;
        st7701s_handle->spi_io_config_t.quadwp_io_num = -1;
        st7701s_handle->spi_io_config_t.quadhd_io_num = -1;

        st7701s_handle->spi_io_config_t.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;

        ESP_ERROR_CHECK(spi_bus_initialize(channel_select, &(st7701s_handle->spi_io_config_t),SPI_DMA_CH_AUTO));

        st7701s_handle->st7701s_protocol_config_t.command_bits = 1;
        st7701s_handle->st7701s_protocol_config_t.address_bits = 8;
        st7701s_handle->st7701s_protocol_config_t.clock_speed_hz = 10000000;
        st7701s_handle->st7701s_protocol_config_t.mode = 0;
        st7701s_handle->st7701s_protocol_config_t.spics_io_num = CS;
        st7701s_handle->st7701s_protocol_config_t.queue_size = 1;

        ESP_ERROR_CHECK(spi_bus_add_device(channel_select, &(st7701s_handle->st7701s_protocol_config_t),
                                        &(st7701s_handle->spi_device)));
        
        return st7701s_handle;
    }else{
        ioexpander_init();
    }
    return NULL;
}

/**
 * @brief Screen initialization
 * @param St7701S_handle 
 * @param type 
 * @note
*/
void ST7701S_screen_init(ST7701S_handle St7701S_handle, unsigned char type)
{
    if (type == 1){
    // 2.8inch
    SPI_WriteComm(0xFF);     
    SPI_WriteData(0x77);   
    SPI_WriteData(0x01);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x13);   

    SPI_WriteComm(0xEF);     
    SPI_WriteData(0x08);   

    SPI_WriteComm(0xFF);     
    SPI_WriteData(0x77);   
    SPI_WriteData(0x01);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x10);   

    SPI_WriteComm(0xC0);     
    SPI_WriteData(0x3B);   
    SPI_WriteData(0x00);   

    SPI_WriteComm(0xC1);     
    SPI_WriteData(0x10);   
    SPI_WriteData(0x0C);   

    SPI_WriteComm(0xC2);     
    SPI_WriteData(0x07);   
    SPI_WriteData(0x0A);   

    SPI_WriteComm(0xC7);     
    SPI_WriteData(0x00);   

    SPI_WriteComm(0xCC);     
    SPI_WriteData(0x10);   

    SPI_WriteComm(0xCD);     
    SPI_WriteData(0x08); 

    SPI_WriteComm(0xB0);     
    SPI_WriteData(0x05);   
    SPI_WriteData(0x12);   
    SPI_WriteData(0x98);   
    SPI_WriteData(0x0E);   
    SPI_WriteData(0x0F);   
    SPI_WriteData(0x07);   
    SPI_WriteData(0x07);   
    SPI_WriteData(0x09);   
    SPI_WriteData(0x09);   
    SPI_WriteData(0x23);   
    SPI_WriteData(0x05);   
    SPI_WriteData(0x52);   
    SPI_WriteData(0x0F);   
    SPI_WriteData(0x67);   
    SPI_WriteData(0x2C);   
    SPI_WriteData(0x11);   

    SPI_WriteComm(0xB1);     
    SPI_WriteData(0x0B);   
    SPI_WriteData(0x11);   
    SPI_WriteData(0x97);   
    SPI_WriteData(0x0C);   
    SPI_WriteData(0x12);   
    SPI_WriteData(0x06);   
    SPI_WriteData(0x06);   
    SPI_WriteData(0x08);   
    SPI_WriteData(0x08);   
    SPI_WriteData(0x22);   
    SPI_WriteData(0x03);   
    SPI_WriteData(0x51);   
    SPI_WriteData(0x11);   
    SPI_WriteData(0x66);   
    SPI_WriteData(0x2B);   
    SPI_WriteData(0x0F);   

    SPI_WriteComm(0xFF);     
    SPI_WriteData(0x77);   
    SPI_WriteData(0x01);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x11);   

    SPI_WriteComm(0xB0);     
    SPI_WriteData(0x5D);   

    SPI_WriteComm(0xB1);     
    SPI_WriteData(0x3E);   

    SPI_WriteComm(0xB2);     
    SPI_WriteData(0x81);   

    SPI_WriteComm(0xB3);     
    SPI_WriteData(0x80);   

    SPI_WriteComm(0xB5);     
    SPI_WriteData(0x4E);   

    SPI_WriteComm(0xB7);     
    SPI_WriteData(0x85);   

    SPI_WriteComm(0xB8);     
    SPI_WriteData(0x20);   

    SPI_WriteComm(0xC1);     
    SPI_WriteData(0x78);   

    SPI_WriteComm(0xC2);     
    SPI_WriteData(0x78);  

    SPI_WriteComm(0xD0);     
    SPI_WriteData(0x88);   

    SPI_WriteComm(0xE0);     
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x02);   

    SPI_WriteComm(0xE1);     
    SPI_WriteData(0x06);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0x08);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0x05);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0x07);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x33);   
    SPI_WriteData(0x33);   

    SPI_WriteComm(0xE2);     
    SPI_WriteData(0x11);   
    SPI_WriteData(0x11);   
    SPI_WriteData(0x33);   
    SPI_WriteData(0x33);   
    SPI_WriteData(0xF4);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0xF4);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   

    SPI_WriteComm(0xE3);     
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x11);   
    SPI_WriteData(0x11);   

    SPI_WriteComm(0xE4);     
    SPI_WriteData(0x44);   
    SPI_WriteData(0x44);   

    SPI_WriteComm(0xE5);     
    SPI_WriteData(0x0D);   
    SPI_WriteData(0xF5);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   
    SPI_WriteData(0x0F);   
    SPI_WriteData(0xF7);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   
    SPI_WriteData(0x09);   
    SPI_WriteData(0xF1);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   
    SPI_WriteData(0x0B);   
    SPI_WriteData(0xF3);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   

    SPI_WriteComm(0xE6);     
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x11);   
    SPI_WriteData(0x11);   

    SPI_WriteComm(0xE7);     
    SPI_WriteData(0x44);   
    SPI_WriteData(0x44);   

    SPI_WriteComm(0xE8);     
    SPI_WriteData(0x0C);   
    SPI_WriteData(0xF4);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   
    SPI_WriteData(0x0E);   
    SPI_WriteData(0xF6);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   
    SPI_WriteData(0x08);   
    SPI_WriteData(0xF0);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   
    SPI_WriteData(0x0A);   
    SPI_WriteData(0xF2);   
    SPI_WriteData(0x30);   
    SPI_WriteData(0xF0);   

    SPI_WriteComm(0xE9);     
    SPI_WriteData(0x36);   
    SPI_WriteData(0x01);   

    SPI_WriteComm(0xEB);     
    SPI_WriteData(0x00);   
    SPI_WriteData(0x01);   
    SPI_WriteData(0xE4);   
    SPI_WriteData(0xE4);   
    SPI_WriteData(0x44);   
    SPI_WriteData(0x88);   
    SPI_WriteData(0x40);   

    SPI_WriteComm(0xED);     
    SPI_WriteData(0xFF);   
    SPI_WriteData(0x10);   
    SPI_WriteData(0xAF);   
    SPI_WriteData(0x76);   
    SPI_WriteData(0x54);   
    SPI_WriteData(0x2B);   
    SPI_WriteData(0xCF);   
    SPI_WriteData(0xFF);   
    SPI_WriteData(0xFF);   
    SPI_WriteData(0xFC);   
    SPI_WriteData(0xB2);   
    SPI_WriteData(0x45);   
    SPI_WriteData(0x67);   
    SPI_WriteData(0xFA);   
    SPI_WriteData(0x01);   
    SPI_WriteData(0xFF);   

    SPI_WriteComm(0xEF);     
    SPI_WriteData(0x08);   
    SPI_WriteData(0x08);   
    SPI_WriteData(0x08);   
    SPI_WriteData(0x45);   
    SPI_WriteData(0x3F);   
    SPI_WriteData(0x54);   

    SPI_WriteComm(0xFF);     
    SPI_WriteData(0x77);   
    SPI_WriteData(0x01);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   
    SPI_WriteData(0x00);   

    // SPI_WriteComm(0x0C);     
    // SPI_WriteData(0x60);  

    SPI_WriteComm(0x11);     
    Delay(120);  

    SPI_WriteComm(0x3A);     
    SPI_WriteData(0x66);   

    SPI_WriteComm(0x36);     
    SPI_WriteData(0x00);   

    SPI_WriteComm(0x35);     
    SPI_WriteData(0x00);   

    SPI_WriteComm(0x20); 
    Delay (120);
    SPI_WriteComm(0x29);   
    }
}

/**
 * @brief Example Delete the ST7701S object
 * @param St7701S_handle 
*/
void ST7701S_delObject(ST7701S_handle St7701S_handle)
{
    assert(St7701S_handle != NULL);
    free(St7701S_handle);
}

/**
 * @brief SPI write instruction
 * @param St7701S_handle 
 * @param cmd instruction
*/
void ST7701S_WriteCommand(ST7701S_handle St7701S_handle, uint8_t cmd)
{
    if(St7701S_handle->method_select){
        spi_transaction_t spi_tran = {
            .rxlength = 0,
            .length = 0,
            .cmd = 0,
            .addr = cmd,
        };
        spi_device_transmit(St7701S_handle->spi_device, &spi_tran);
    }else{
        ioexpander_write_cmd();
    }
}

/**
 * @brief SPI write data
 * @param St7701S_handle
 * @param data 
*/
void ST7701S_WriteData(ST7701S_handle St7701S_handle, uint8_t data)
{
    if(St7701S_handle->method_select){
        spi_transaction_t spi_tran = {
            .rxlength = 0,
            .length = 0,
            .cmd = 1,
            .addr = data,
        };
        spi_device_transmit(St7701S_handle->spi_device, &spi_tran);
    }else{
        ioexpander_write_data();
    }
}
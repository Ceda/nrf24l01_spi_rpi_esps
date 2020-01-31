#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ringbuf.h"

#include "esp8266/spi_struct.h"
#include "esp8266/gpio_struct.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi.h"


#define NRF24L01_CE_GPIO     4
#define NRF24L01_CE_MASK    (1ULL<<NRF24L01_CE_GPIO)

void d1_gpio_init(){
    printf( "init gpio\n");
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = NRF24L01_CE_MASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void d1_spi_init(){
    printf( "init spi\n");
    spi_config_t spi_config;
    // Load default interface parameters
    // CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
    // Load default interrupt enable
    // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    // Set SPI to master mode
    // ESP8266 Only support half-duplex
    spi_config.mode = SPI_MASTER_MODE;
    // Set the SPI clock frequency division factor
    spi_config.clk_div = 800;   //100khz     val=40 = 2mhz
    spi_config.event_cb = NULL;
    spi_init(HSPI_HOST, &spi_config);
}

void spi_read_bytes ( uint16_t cmd, uint8_t *rdata, int length){
     uint32_t rx[16];
     spi_trans_t trans;
     memset(&trans, 0x0, sizeof(trans));
     trans.bits.val = 0;
     trans.cmd = &cmd;
     trans.miso = rx;
     trans.addr = NULL;
     trans.bits.cmd = 8 * 1;   
     trans.bits.miso = 8 * length; 
     spi_trans(HSPI_HOST, &trans);

     for (int n = 0; n <= length/4; n++) *(uint32_t*) &rdata[4*n] = rx[n];
}    

void spi_write_bytes ( uint16_t cmd, uint8_t *wdata, int length){
     uint32_t wx[16];
     for(int n=0; n< length; n++) wx[n] = *(uint32_t*) &wdata[4*n];

     spi_trans_t trans;
     memset(&trans, 0x0, sizeof(trans));
     trans.bits.val = 0;
     trans.bits.cmd = 8 * 1;  
     trans.bits.addr = 0;          // transmit status do not use address bit
     trans.bits.mosi = 8 * length;
     trans.cmd = &cmd;
     trans.addr = NULL;
     trans.mosi = wx;
     spi_trans(HSPI_HOST, &trans);    
}

void app_main(void)
{
    printf("hi world\n");

    d1_gpio_init();
    d1_spi_init();

    uint32_t regaddr = 0x03;
    uint16_t cmd = regaddr;
    uint8_t rdata[64];
    int length = 7;
         
    while(1) {
        spi_read_bytes ( cmd, rdata, length);

	for(int a=0; a<length; a++) printf("0x%02x ",rdata[a]);
	printf("\n");

        vTaskDelay(50);
    }

}

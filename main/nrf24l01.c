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
#include <esp_timer.h>

//spi dependancies
#define NRF24L01_CE_GPIO    16
#define NRF24L01_CE_MASK    (1ULL<<NRF24L01_CE_GPIO)
#include "./spi.h"

//i2c depenencies
#define I2C_SCL_IO           5                /*!< gpio number for I2C master clock */
#define I2C_SDA_IO           4                /*!< gpio number for I2C master data  */
#include "./i2c.h"
#include "./ssd1306.h"

int cnt = 0;

int nrf24_transmit_pkt ( uint8_t *data, int length) {
    //setup nrf24l01 transmitter
    gpio_set_level (NRF24L01_CE_GPIO, 0);
    spi_write_byte ( 0x20 | 0x00, 0x00); //no crc, tx mode
    spi_write_byte ( 0x20 | 0x01, 0x00); //no auto ack
    spi_write_byte ( 0x20 | 0x02, 0x01); //pipe0
    spi_write_byte ( 0x20 | 0x03, 0x03);
    spi_write_byte ( 0x20 | 0x04, 0x04);
    spi_write_byte ( 0x20 | 0x05, 0x05); //freq channel 5
    spi_write_byte ( 0x20 | 0x06, 0x06); //low power, 1MB/sec
    spi_write_byte ( 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spi_write_byte ( 0x20 | 0x00, 0x02); //turn on
    spi_write_byte ( 0xe1, 0x00);        //flush tx fifo
    spi_write_byte ( 0x20 | 0x07, 0x70);
    ets_delay_us(10);                          //busy-wait

    //vTaskDelay(1);
    //printf("   post flush  = 0x%02x  0x%02x\n", 
    //      spi_read_bytes ( 0x07, data, 1), spi_read_bytes ( 0x17, data, 1));

    //send packet to nrf24l01 transmitter
    spi_write_bytes ( 0xa0, data, length);

    //vTaskDelay(1);
    //printf("   post fifow  = 0x%02x  0x%02x\n", 
    //      spi_read_bytes ( 0x07, data, 1), spi_read_bytes ( 0x17, data, 1));

    //ce chip radio enable
    //with jumper wires on bread board ce pulse 45us was about 50%
    gpio_set_level (NRF24L01_CE_GPIO, 1);
    ets_delay_us(500);                          //busy-wait
    gpio_set_level (NRF24L01_CE_GPIO, 0);

    //vTaskDelay(1);
    //printf("   post trans  = 0x%02x  0x%02x\n", 
    //      spi_read_bytes ( 0x07, data, 1), spi_read_bytes ( 0x17, data, 1));

    spi_write_byte ( 0x20 | 0x00, 0x00); //turn off

    return(0);
}

int wait_rcv_pkt ( uint8_t *data, int timeout) {
    //setup nrf24l01 transmitter
    gpio_set_level (NRF24L01_CE_GPIO, 0);
    spi_write_byte ( 0x20 | 0x00, 0x00); //no crc, tx mode
    spi_write_byte ( 0x20 | 0x00, 0x01); //no crc, rx mode
    spi_write_byte ( 0x20 | 0x01, 0x00); //no auto ack
    spi_write_byte ( 0x20 | 0x02, 0x01); //pipe0
    spi_write_byte ( 0x20 | 0x03, 0x03);
    spi_write_byte ( 0x20 | 0x04, 0x00);
    spi_write_byte ( 0x20 | 0x05, 0x05); //freq channel 5
    spi_write_byte ( 0x20 | 0x06, 0x06); //low power, 1MB/sec
    spi_write_byte ( 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spi_write_byte ( 0x20 | 0x00, 0x03); //turn on
    spi_write_byte ( 0xe2, 0x00); //flush rx fifo
    spi_write_byte ( 0x20 | 0x07, 0x70);

    gpio_set_level (NRF24L01_CE_GPIO, 1);
    ets_delay_us(100);                          //busy-wait

    int waitcnt = 0;
    int timestart = esp_timer_get_time();
    while(1){
	spi_read_bytes ( 0x07, data, 1);
        //ets_delay_us(1000);                          //busy-wait
	vTaskDelay(1);
        if( (data[0] & 0x40) > 1 || waitcnt > timeout) break;
        ++waitcnt;
	//vTaskDelay(1);
    }
    //if (waitcnt > timeout) printf("wait timed out\n");
    if(waitcnt < timeout) spi_read_bytes ( 0x61, data, 32+1);

    //set ce = 0
    gpio_set_level (NRF24L01_CE_GPIO, 0);

    printf("waited %8.4fsec   ", 
    	    (float)(esp_timer_get_time()-timestart)/1000000);
    return(waitcnt);
}

void rflink_task () {
    uint8_t data[33];
    TickType_t xLoopStart = xTaskGetTickCount();
    while(1) {
	sprintf((char*) data, "%4d,%4d,%4d,%4d,%4d,%2d", cnt,1000,1500,1500,1500,7);
	//printf("%s", data);
        nrf24_transmit_pkt ( (uint8_t*)data, 32 );
	++cnt;

	int timeout = 5;  //1msec per
        int waitlen = wait_rcv_pkt ( (uint8_t*)data, timeout); 
	if (waitlen < timeout)
            for(int a =0; a<32; a++) printf("0x%02x ", data[a]); printf("\n");

	vTaskDelay(1);
	vTaskDelayUntil(&xLoopStart, 10);
    }
}

void app_main(void)
{
    uint8_t data[33] = { 'H', 'e', 'l', 'l', 'o', '\n' }; //max length nrf20l01 packet

    gpio_initialize();
    spi_initialize();
    nrf24_transmit_pkt ( data, 6 );

    i2c_init();
    i2c_detect();

    ssd1305_init();
    ssd1305_blank(0x00);

    char disp_str[128] = "4 Hi There";
    ssd1305_text(disp_str);

    //print register space
    for (int a = 0; a < 0x1d; a++)
	printf(" reg 0x%02x  data 0x%02x\n", a, spi_read_bytes ( a, data, 1));

    xTaskCreate (rflink_task, "rflink_task", 2048, NULL, 4, NULL);

    TickType_t xLoopStart = xTaskGetTickCount();
    while(1) {
	sprintf (disp_str, "4 F-450 Xmit|||1 height=%3d| 0x%02x",cnt, data[1]);
        ssd1305_text(disp_str);
	vTaskDelay(100);
    }

}

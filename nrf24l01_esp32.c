#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "driver/spi_common.h"
#include "sdkconfig.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "esp32/rom/ets_sys.h"

//requirements for spi
#define MISO_PIN  17
#define MOSI_PIN  5
#define SCLK_PIN  23
#define CS_PIN    16
#define SPI_CLOCK 100000  // 500KHz may be a bit too fast for breadboard
#include "./spi.h"

#define GPIO_NRF24L01_CE        22
#define GPIO_NRF24L01_CE_MASK   (1ULL << 22)


int nrf24_receive_pkt ( uint8_t *data, int length) {
    //setup nrf24l01 transmitter
    spiWriteByte (spi, 0x20 | 0x00, 0x01); //no crc, rx mode
    spiWriteByte (spi, 0x20 | 0x01, 0x00); //no auto ack
    spiWriteByte (spi, 0x20 | 0x02, 0x01); //pipe0
    spiWriteByte (spi, 0x20 | 0x03, 0x03); 
    spiWriteByte (spi, 0x20 | 0x04, 0x00); 
    spiWriteByte (spi, 0x20 | 0x05, 0x05); //freq channel 5
    spiWriteByte (spi, 0x20 | 0x06, 0x06); //low power, 1MB/sec
    spiWriteByte (spi, 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spiWriteByte (spi, 0x20 | 0x00, 0x03); //turn on
    spiWriteByte (spi, 0xe1, 0x00); //flush tx fifo
    spiWriteByte (spi, 0x20 | 0x07, 0x70); 
    gpio_set_level (GPIO_NRF24L01_CE, 1);
    ets_delay_us(100);                          //busy-wait

    int waitcnt = 0;
    while(1){
	spiReadByte (spi, 0x07, data);
        int temp = data[0];
        ets_delay_us(10000);
        //printf(" %d    0x%x\n", waitcnt, temp);
        if( (temp & 0x40) > 1 || waitcnt > 200) break;
        ++waitcnt;
    }
    if (waitcnt == 20) printf("wait timed out\n");

    //read packet from nrf24l01 transmitter
    spiReadBytes ( spi, 0x61, 32+1, data);

    //set ce = 0
    gpio_set_level (GPIO_NRF24L01_CE, 0);

    printf("%8.4f   waited %3dmsec  for %2dbytes\n      ", (float)esp_timer_get_time()/1000000, 10*waitcnt, 32);
    for(int n = 0; n < 32; n++) {printf(" 0x%02x", data[n+1]); if(n%16 == 15) printf("\n      "); }

    vTaskDelay(1);
    return(0);
}

int nrf24_transmit_pkt ( uint8_t *data, int length) {
    //setup nrf24l01 transmitter
    spiWriteByte (spi, 0x20 | 0x00, 0x00); //no crc, tx mode
    spiWriteByte (spi, 0x20 | 0x01, 0x00); //no auto ack
    spiWriteByte (spi, 0x20 | 0x02, 0x01); //pipe0
    spiWriteByte (spi, 0x20 | 0x03, 0x03); 
    spiWriteByte (spi, 0x20 | 0x04, 0x04); 
    spiWriteByte (spi, 0x20 | 0x05, 0x05); //freq channel 5
    spiWriteByte (spi, 0x20 | 0x06, 0x06); //low power, 1MB/sec
    spiWriteByte (spi, 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spiWriteByte (spi, 0x20 | 0x00, 0x02); //turn on
    spiWriteByte (spi, 0xe1, 0x00); //flush tx fifo
    spiWriteByte (spi, 0x20 | 0x07, 0x70); 
    ets_delay_us(100);                          //busy-wait

    //send packet to nrf24l01 transmitter
    spiWriteBytes (spi, 0xa0, length, data);

        //ce chip radio enable
    //with jumper wires on bread board ce pulse 45us was about 50%
    gpio_set_level (GPIO_NRF24L01_CE, 1);
    ets_delay_us(500);                          //busy-wait
    gpio_set_level (GPIO_NRF24L01_CE, 0);

    spiWriteByte (spi, 0x20 | 0x00, 0x00); //turn off

    vTaskDelay(1);
    return(0);
}

void gpio_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_NRF24L01_CE);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config ( &io_conf );
}

void app_main(void)
{
    gpio_init();
    spi_init();

    uint8_t data[32];
    for(int i = 0; i <= 0x1d; i++){
       spiReadByte (spi, i, data);
       printf("regaddr = 0x%02x  data = 0x%02x\n", i, data[0]);
    }

    int cnt = 0;

    while(1) {
	 for(int a=0; a < 32; a++) data[a] = cnt + a;
         nrf24_transmit_pkt (data, 32);
	 ets_delay_us(1000000);
	 printf("%7.4f  sent packet\n", (float) esp_timer_get_time() / 1000000);
         ++cnt;
    }


}

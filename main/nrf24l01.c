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

//spi dependancies
#define NRF24L01_CE_GPIO     0
#define NRF24L01_CE_MASK    (1ULL<<NRF24L01_CE_GPIO)
#include "./spi.h"

void app_main(void)
{
    uint8_t data[64]; //use as nrf24l01 bidirectional data

    d1_gpio_init();
    d1_spi_init();

    //print register space
    for (int a = 0; a < 0x1d; a++){
        spi_read_bytes ( a, data, 1);
	printf(" reg 0x%02x  data 0x%02x\n", a, data[0]);
    }

    data[0] = 0x00;
    spi_write_bytes ( 0x20 | 0x00, data, 1);

    //print register space
    for (int a = 0; a < 0x1d; a++)
	printf(" reg 0x%02x  data 0x%02x\n", a, spi_read_bytes ( a, data, 1));
    

}

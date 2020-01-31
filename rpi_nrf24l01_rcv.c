// gcc -Wall -pthread -o bb_spi_x_test bb_spi_x_test.c -lpigpiod_if2
// ./bb_spi_x_test

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "pigpiod_if2.h"

#define MISO 9 
#define CE0  8
#define MOSI 10
#define SCLK 11
#define CE_NRF  17

//spi_write and spi_read appear not implemented
int pi;
unsigned char inBuf[3];

void spi_write_reg(int reg, int value){
   char cmdstr[] = {reg, value}; 
   bb_spi_xfer(pi, CE0, cmdstr, (char *)inBuf, 2); 
}

int spi_read_reg(int reg){
   char cmdstr[] = {reg, 0}; 
   bb_spi_xfer(pi, CE0, cmdstr, (char *)inBuf, 2); 
   return inBuf[1];
}

void spi_read_bytes(int reg, int length, char *data){
   char cmdstr[] = {reg, 0}; 
   bb_spi_xfer(pi, CE0, cmdstr, (char *)data, length); 
}

//need spi_write_bytes tested
void spi_write_bytes(int reg, int length, char *data){
   char cmdstr[] = {reg, data}; //not tested may noe unpack??
   bb_spi_xfer(pi, CE0, cmdstr, (char *)data, length); 
}

//need spi_write_bytes tested

double timer(){
   struct timespec ts;
   clock_gettime(CLOCK_REALTIME, &ts);
   return (  ((double)ts.tv_nsec/1000000000) +  ts.tv_sec );
}

int main(int argc, char *argv[])
{
    char data[32];
    double startt = timer();   
    //set ce = 0
    set_mode (pi, CE_NRF, PI_OUTPUT);
    gpio_write (pi, CE_NRF, 0);

    // interface to running pigpiod daemon
    if ((pi = pigpio_start(0, 0)) < 0) { 
        fprintf(stderr, "pigpio init failed (%d).\n", pi);
        fprintf(stderr, "    is pigpiod & running?");
    }

    //open spi and get handle at freq and mode
    bb_spi_open(pi, CE0, MISO, MOSI, SCLK, 100000, 0); // nrf24l01+

    int cnt = 0; 
    int n;
    while(1) {
        spi_write_reg (0x20 | 0x00, 0x01); //no crc, rx mode
        spi_write_reg (0x20 | 0x01, 0x00); //pipe0 no auto ack
        spi_write_reg (0x20 | 0x02, 0x03); //pipe0
        spi_write_reg (0x20 | 0x03, 0x03);
        spi_write_reg (0x20 | 0x04, 0x00);
        spi_write_reg (0x20 | 0x05, 0x05); //freq channel 5
        spi_write_reg (0x20 | 0x06, 0x02); //low power, 1MB/sec
        spi_write_reg (0x20 | 0x11, 0x20); //use all 32 bytes
        spi_write_reg (0x20 | 0x00, 0x03); //power up, no crc, rx mode
        spi_write_reg ( 0xe1, 0x00);i      //flush rx fifo
        spi_write_reg (0x20 | 0x07, 0x70);
        //set ce = 1
        gpio_write (pi, CE_NRF, 1);

        int waitcnt = 0;
        while(1){
            int temp = spi_read_reg(0x07);
            usleep(10000);
            //printf(" %d    0x%x\n", waitcnt, temp);
            if( (temp & 0x40) > 1 || waitcnt > 200) break;
            ++waitcnt;

double timer(){
   struct timespec ts;
   clock_gettime(CLOCK_REALTIME, &ts);
   return (  ((double)ts.tv_nsec/1000000000) +  ts.tv_sec );
}

int main(int argc, char *argv[])
{
    char data[32];
    double startt = timer();   
    //set ce = 0
    set_mode (pi, CE_NRF, PI_OUTPUT);
    gpio_write (pi, CE_NRF, 0);

    // interface to running pigpiod daemon
    if ((pi = pigpio_start(0, 0)) < 0) { 
        fprintf(stderr, "pigpio init failed (%d).\n", pi);
        fprintf(stderr, "    is pigpiod & running?");
    }

    //open spi and get handle at freq and mode
    bb_spi_open(pi, CE0, MISO, MOSI, SCLK, 100000, 0); // nrf24l01+

    int cnt = 0; 
    int n;
    while(1) {
        spi_write_reg (0x20 | 0x00, 0x01); //no crc, rx mode
        spi_write_reg (0x20 | 0x01, 0x00); //pipe0 no auto ack
        spi_write_reg (0x20 | 0x02, 0x03); //pipe0
        spi_write_reg (0x20 | 0x03, 0x03);
        spi_write_reg (0x20 | 0x04, 0x00);
        spi_write_reg (0x20 | 0x05, 0x05); //freq channel 5
        spi_write_reg (0x20 | 0x06, 0x02); //low power, 1MB/sec
        spi_write_reg (0x20 | 0x11, 0x20); //use all 32 bytes
        spi_write_reg (0x20 | 0x00, 0x03); //power up, no crc, rx mode
        spi_write_reg ( 0xe1, 0x00);i      //flush rx fifo
        spi_write_reg (0x20 | 0x07, 0x70);
        //set ce = 1
        gpio_write (pi, CE_NRF, 1);

        int waitcnt = 0;
        while(1){
            int temp = spi_read_reg(0x07);
            usleep(10000);
            //printf(" %d    0x%x\n", waitcnt, temp);
            if( (temp & 0x40) > 1 || waitcnt > 200) break;
            ++waitcnt;
        }
        if (waitcnt == 20) printf("wait timed out\n");

        //read packet from nrf24l01 transmitter
        spi_read_bytes ( 0x61, 32+1, data);

        //set ce = 0
        gpio_write (pi, CE_NRF, 0);

        printf("%8.4f   waited %3dmsec  for %2dbytes\n      ", 
			timer() - startt, 10*waitcnt, data[0]);
        for(n = 0; n < 32; n++) {
            printf(" 0x%02x", data[n+1]); 
            if(n%16 == 15) printf("\n      ");
       	}
        printf ("\n");
    }
   
    //clean up
    bb_spi_close(pi, CE0);
    pigpio_stop(pi);
    return (0);
}

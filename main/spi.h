//esp8266 spi subroutines
//     spi_initialize()
//     spi_read_bytes()  - also returns first byte
//     spi_write_bytes()

void gpio_initialize(){
    printf( "init gpio\n");
    gpio_config_t io_conf;
    io_conf.intr_type = NRF24L01_CE_GPIO;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = NRF24L01_CE_MASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void spi_initialize(){
    printf( "init spi\n");
    spi_config_t spi_config;
    // Load default interface parameters
    // CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, 
    // BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
    // Load default interrupt enable
    // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, i
    // WRITE_BUFFER: false, READ_BUFFER: false
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    // Set SPI to master mode
    // ESP8266 Only support half-duplex (and non-mappable interface)
    spi_config.mode = SPI_MASTER_MODE;
    // Set the SPI clock frequency division factor (divide from 80MHz)
    spi_config.clk_div = 800;   //100khz     val=40 = 2mhz
    spi_config.event_cb = NULL;
    spi_init(HSPI_HOST, &spi_config);
}

uint8_t spi_read_bytes ( uint16_t cmd, uint8_t *rdata, int length){
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
     
     //convert to byte array from uint32_t array gathered at miso
     for (int n = 0; n <= length/4; n++) *(uint32_t*) &rdata[4*n] = rx[n];
     return rdata[0];
}    

void spi_write_bytes ( uint16_t cmd, uint8_t *wdata, int length){
     uint32_t wx[16];
     //convert to uint32_t array from passed byte array
     for(int n=0; n< length; n=n+4) wx[n/4] = *(uint32_t*) &wdata[n];

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

void spi_write_byte ( uint16_t cmd, uint32_t data){
     spi_trans_t trans;
     memset(&trans, 0x0, sizeof(trans));
     trans.bits.val = 0;
     trans.bits.cmd = 8 * 1;  
     trans.bits.addr = 0;          // transmit status do not use address bit
     trans.bits.mosi = 8 * 1;
     trans.cmd = &cmd;
     trans.addr = NULL;
     trans.mosi = &data;
     spi_trans(HSPI_HOST, &trans);    
}


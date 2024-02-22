#include <string.h>
#include <stdint.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "PMW3360.h"
#include "PMW3360_SROM_0X04.h"

#define TAG "PMW3360"

static spi_host_device_t HOST_ID  = SPI3_HOST;
static spi_device_handle_t SPIHandle;
static esp_err_t ret;

static SemaphoreHandle_t spi_bus_mutex;

static volatile Motion_Data Sensor1_Data = {
    .cs_pin = PMW3360_CS0,
    .motion = 0.0,
    .dx = 0.0,
    .dy = 0.0,
    .x_dist = 0.0, //in cm
    .y_dist = 0.0, //in cm
    .timestamps = 0, // in ms
};

static volatile Motion_Data Sensor2_Data = {
    .cs_pin = PMW3360_CS1,
    .motion = 0.0,
    .dx = 0.0,
    .dy = 0.0,
    .x_dist = 0.0, //in cm
    .y_dist = 0.0, //in cm
    .timestamps = 0, // in ms
};

static volatile Motion_Data Sensor3_Data = {
    .cs_pin = PMW3360_CS2,
    .motion = 0,
    .dx = 0,
    .dy = 0,
    .x_dist = 0, //in cm
    .y_dist = 0, //in cm
    .timestamps = 0, // in ms
};

void pmw3360_spi_init(int16_t GPIO_SCLK, int16_t GPIO_MISO, int16_t GPIO_MOSI){

    spi_bus_mutex = xSemaphoreCreateMutex();

	gpio_reset_pin( Sensor1_Data.cs_pin );
	gpio_set_direction( Sensor1_Data.cs_pin, GPIO_MODE_OUTPUT );

	gpio_reset_pin( Sensor2_Data.cs_pin );
	gpio_set_direction( Sensor2_Data.cs_pin, GPIO_MODE_OUTPUT );

    gpio_reset_pin( Sensor3_Data.cs_pin );
	gpio_set_direction( Sensor3_Data.cs_pin, GPIO_MODE_OUTPUT );

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = GPIO_MOSI;
    buscfg.miso_io_num = GPIO_MISO;
    buscfg.sclk_io_num = GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.mode = 3; //SPI_MODE;
    devcfg.duty_cycle_pos = 128; // was 0
    devcfg.cs_ena_pretrans = 0;
    devcfg.cs_ena_posttrans = 0; //CS_ENA_POSTTRANS;
    devcfg.clock_speed_hz = PMW_3360_SPI_Frequency; //SPI_HZ;
    devcfg.input_delay_ns = 0; // INPUT_DELAY_NS; was 50
    // devcfg.spics_io_num = GPIO_CS;
    devcfg.flags = 0 ; // was SPI_DEVICE_NO_DUMMY
    devcfg.queue_size = 7;
    devcfg.pre_cb = 0;
    devcfg.post_cb = 0;

    ret = spi_bus_initialize(HOST_ID, &buscfg, 0); // No DMA ,SPI_DMA_CH_AUTO
    ESP_ERROR_CHECK(ret);
    ret =  spi_bus_add_device(HOST_ID, &devcfg, &SPIHandle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP32 device initialized on SPI with MISO=%d, MOSI=%d, CLK=%d HZ=%d", GPIO_MISO, GPIO_MOSI, GPIO_SCLK, PMW_3360_SPI_Frequency);
}

static void delay_ms(uint32_t delay){
     vTaskDelay(delay / portTICK_PERIOD_MS);
}

#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}
static void IRAM_ATTR delay_us(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

static void spi_begin(const uint8_t cs_num){
	gpio_set_level( cs_num, 0 );
}

static void spi_end(const uint8_t cs_num){
	gpio_set_level( cs_num, 1 );
}

static uint8_t spi_write_read(const uint8_t data){

    spi_transaction_t trans;
    uint8_t rx_data;

    memset(&trans, 0, sizeof(trans));
    trans.length = 8;
    trans.rxlength = 8;
    trans.tx_buffer = &data;
    trans.rx_buffer = &rx_data;

    spi_device_polling_transmit(SPIHandle, &trans);
    
    return rx_data;
}

static uint8_t spi_read_reg(const uint8_t cs_num, const uint8_t reg_addr){
    // printf("Read From %x\n",reg_addr & 0x7f);

    uint8_t rx_data;
    spi_begin(cs_num);
    spi_write_read(reg_addr & 0x7f);

    rx_data = spi_write_read(0x00);
    delay_us(1);
    spi_end(cs_num);
    delay_us(19);

    // printf("Read: %x \n", rx_data);
    return rx_data;
}

static void spi_write_reg(const uint8_t cs_num, const uint8_t reg_addr, uint8_t data)
{
    // printf("Write %x\n", (reg_addr | 0x80) << 8 | data);
    // uint8_t tx_data[2] = {reg_addr | 0x80, data};
    // uint16_t tx_data = (reg_addr | 0x80) << 8 | data;

    spi_begin(cs_num);
    // spi_write_read(tx_data);
    spi_write_read(reg_addr | 0x80);
    spi_write_read(data);
    delay_us(20);
    spi_end(cs_num);
    delay_us(100);
}


void performStartup(const uint8_t cs_num){
    spi_end(cs_num);
    spi_begin(cs_num);
    spi_end(cs_num);

    spi_write_reg(cs_num, Shutdown, 0xb6); // force reset
    delay_ms(300);

    spi_begin(cs_num);
    delay_us(40);
    spi_end(cs_num);
    delay_us(40);

    spi_write_reg(cs_num ,Power_Up_Reset, 0x5a); // force reset
    delay_ms(50);


    spi_read_reg(cs_num,Product_ID);
    spi_read_reg(cs_num,Revision_ID);
    spi_read_reg(cs_num,Inverse_Product_ID);
    // // read registers 0x02 to 0x06 (and discard the data)
    // // volatile uint16_t temp[5];

    spi_read_reg(cs_num,Motion);
    spi_read_reg(cs_num,Delta_X_L);
    spi_read_reg(cs_num,Delta_X_H);
    spi_read_reg(cs_num,Delta_Y_L);
    spi_read_reg(cs_num,Delta_Y_H);

    // upload the firmware
    upload_firmware(cs_num);
    delay_ms(10);

    setCPI(cs_num,CPI);
    ESP_LOGI(TAG, "Optical Chip Initialized");
}

static int constrain(int value, int min_val, int max_val) {
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    } else {
        return value;
    }
}

void setCPI(const uint8_t cs_num ,int cpi)
{
    int cpival = constrain((cpi/100)-1, 0, 0x77); // limits to 0--119 

    spi_write_reg(cs_num, Config1, cpival);
    uint8_t rx = spi_read_reg(cs_num,Config1);

    ESP_LOGI(TAG, "Set CPI to %d, check: %x\n",cpi, rx);

}

void upload_firmware(const uint8_t cs_num){
    // send the firmware to the chip, cf p.18 of the datasheet
    ESP_LOGI(TAG,"Uploading firmware...");

    //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
    spi_write_reg(cs_num,Config2, 0x00);

    // write 0x1d in SROM_enable reg for initializing
    spi_write_reg(cs_num,SROM_Enable, 0x1d);

    // wait for more than one frame period
    delay_ms(10); // assume that the frame rate is as low as 100fps... even if it should never be that low

    // write 0x18 to SROM_enable to start SROM download
    spi_write_reg(cs_num,SROM_Enable, 0x18);

    spi_begin(cs_num);
    spi_write_read(SROM_Load_Burst | 0x80);
    delay_us(15);

    unsigned char c;
    for (int i = 0; i < firmware_length; i++){
        c = firmware_data[i];
        spi_write_read(c);
        delay_us(15);
    }

    //Read the SROM_ID register to verify the ID before any other register reads or writes.
    // uint16_t SROM_ID_read = spi_read_reg(SROM_ID);
    // printf("Got SROM_ID %x\n", SROM_ID_read);
    uint16_t Product_ID_read = spi_read_reg(cs_num,Product_ID);
    printf("Got Product_ID %x\n", Product_ID_read);
    //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
    spi_write_reg(cs_num,Config2, 0x00);
}

static float CPI2CM(int16_t value){
    //Convert CPI to Inches, then convert Inches to Center meter
    float inches = value / (CPI * 1.0) ;
    float centimeter = inches * 2.54;

    return centimeter;
}

Motion_Data Get_Data(const uint8_t sensor_num){
    if (sensor_num == 0) {return Sensor1_Data;}
    else if (sensor_num == 1) {return Sensor2_Data;}
    else if (sensor_num == 2) {return Sensor3_Data;}

    else {printf("Send Defalut Sensor1 Data\n");}
    
    return Sensor1_Data;
}

void data_collection_task1(){
    volatile uint8_t burstBuffer[12];
    TickType_t startTime = xTaskGetTickCount();
    TickType_t endTime;
    uint8_t cs_ = Sensor1_Data.cs_pin;
    float dx = 0;
    float dy = 0;
    printf("Start Motion Sensor1 Tasks\n");

    for(;;){
        if (xSemaphoreTake(spi_bus_mutex, portMAX_DELAY)){

            // Begin burst transfer by writing to the Motion_Burst register
            spi_write_reg(cs_ ,Motion_Burst, 0);

            // Begin SPI transmission for burst mode and delay 35us (tSRAD_MOTBR)
            spi_begin(cs_);
            spi_write_read(Motion_Burst);
            delay_us(35);

            // Read twelve bytes into the buffer with no delay
            for (int i = 0; i < sizeof(burstBuffer); i++) {
                burstBuffer[i] = spi_write_read(0);
            }
            // Terminate burst transfer and delay 1us (tBEXIT)
            spi_end(cs_);
            delay_us(1);

            // Calculate motion data
            int motion = (burstBuffer[0] & 0x80) != 0;
            dx = (int16_t)(((uint16_t)burstBuffer[3] << 8) + (uint16_t)burstBuffer[2]);
            dy = (int16_t)(((uint16_t)burstBuffer[5] << 8) + (uint16_t)burstBuffer[4]);
            // int surface = (burstBuffer[0] & 0x08) == 0;
            // int SQUAL = burstBuffer[6];
            // int rawDataSum = burstBuffer[7];
            // int maxRawData = burstBuffer[8];
            // int minRawData = burstBuffer[9];
            // int hutter = ((uint16_t)burstBuffer[11] << 8) + (uint16_t)burstBuffer[10];
            if(motion & (dx!=0) & (dy!=0)){
                endTime = xTaskGetTickCount();
                Sensor1_Data.timestamps = endTime - startTime;
                Sensor1_Data.dx = CPI2CM(dx);
                Sensor1_Data.dy = CPI2CM(dy);

                Sensor1_Data.x_dist += Sensor1_Data.dx;
                Sensor1_Data.y_dist += Sensor1_Data.dy;
            }
            // printf("Sensor1 %lld s: dx: %.2f, dy: %.2f, %.2f, %2f\n", Sensor1_Data.timestamps, Sensor1_Data.dx, Sensor1_Data.dy, Sensor1_Data.x_dist, Sensor1_Data.y_dist);

                    
            xSemaphoreGive(spi_bus_mutex);
            }
            delay_ms(10);

        }
}

void data_collection_task2(){
    volatile uint8_t burstBuffer[12];
    TickType_t startTime = xTaskGetTickCount();
    TickType_t endTime;
    uint8_t cs_ = Sensor2_Data.cs_pin;
    float dx = 0;
    float dy = 0;
    printf("Start Motion Sensor2 Tasks\n");

    for(;;){
        
        if (xSemaphoreTake(spi_bus_mutex, portMAX_DELAY)){

            // Begin burst transfer by writing to the Motion_Burst register
            spi_write_reg(cs_ ,Motion_Burst, 0);

            // Begin SPI transmission for burst mode and delay 35us (tSRAD_MOTBR)
            spi_begin(cs_);
            spi_write_read(Motion_Burst);
            delay_us(35);

            // Read twelve bytes into the buffer with no delay
            for (int i = 0; i < sizeof(burstBuffer); i++) {
                burstBuffer[i] = spi_write_read(0);
            }
            // Terminate burst transfer and delay 1us (tBEXIT)
            spi_end(cs_);
            delay_us(1);

            // Calculate motion data
            int motion = (burstBuffer[0] & 0x80) != 0;
            dx = (int16_t)(((uint16_t)burstBuffer[3] << 8) + (uint16_t)burstBuffer[2]);
            dy = (int16_t)(((uint16_t)burstBuffer[5] << 8) + (uint16_t)burstBuffer[4]);
            // int surface = (burstBuffer[0] & 0x08) == 0;
            // int SQUAL = burstBuffer[6];
            // int rawDataSum = burstBuffer[7];
            // int maxRawData = burstBuffer[8];
            // int minRawData = burstBuffer[9];
            // int hutter = ((uint16_t)burstBuffer[11] << 8) + (uint16_t)burstBuffer[10];
            if(motion & (dx!=0) & (dy!=0)){
                endTime = xTaskGetTickCount();
                Sensor2_Data.timestamps = endTime - startTime;
                Sensor2_Data.dx = CPI2CM(dx);
                Sensor2_Data.dy = CPI2CM(dy);

                Sensor2_Data.x_dist += Sensor2_Data.dx;
                Sensor2_Data.y_dist += Sensor2_Data.dy;
            }
            // printf("Sensor2 %lld s: dx: %.2f, dy: %.2f, %.2f, %2f\n", Sensor2_Data.timestamps, Sensor2_Data.dx, Sensor2_Data.dy, Sensor2_Data.x_dist, Sensor2_Data.y_dist);
        
            xSemaphoreGive(spi_bus_mutex);
        }
        delay_ms(10);

    }

}


void data_collection_task3(){
    volatile uint8_t burstBuffer[12];
    TickType_t startTime = xTaskGetTickCount();
    TickType_t endTime;
    uint8_t cs_ = Sensor3_Data.cs_pin;
    float dx = 0;
    float dy = 0;
    printf("Start Motion Sensor3 Tasks\n");

    for(;;){
        if (xSemaphoreTake(spi_bus_mutex, portMAX_DELAY)){

            // Begin burst transfer by writing to the Motion_Burst register
            spi_write_reg(cs_ ,Motion_Burst, 0);

            // Begin SPI transmission for burst mode and delay 35us (tSRAD_MOTBR)
            spi_begin(cs_);
            spi_write_read(Motion_Burst);
            delay_us(35);

            // Read twelve bytes into the buffer with no delay
            for (int i = 0; i < sizeof(burstBuffer); i++) {
                burstBuffer[i] = spi_write_read(0);
            }
            // Terminate burst transfer and delay 1us (tBEXIT)
            spi_end(cs_);
            delay_us(1);

            // Calculate motion data
            int motion = (burstBuffer[0] & 0x80) != 0;
            dx = (int16_t)(((uint16_t)burstBuffer[3] << 8) + (uint16_t)burstBuffer[2]);
            dy = (int16_t)(((uint16_t)burstBuffer[5] << 8) + (uint16_t)burstBuffer[4]);
            // int surface = (burstBuffer[0] & 0x08) == 0;
            // int SQUAL = burstBuffer[6];
            // int rawDataSum = burstBuffer[7];
            // int maxRawData = burstBuffer[8];
            // int minRawData = burstBuffer[9];
            // int hutter = ((uint16_t)burstBuffer[11] << 8) + (uint16_t)burstBuffer[10];
            if(motion & (dx!=0) & (dy!=0)){
                endTime = xTaskGetTickCount();
                Sensor3_Data.timestamps = endTime - startTime;
                Sensor3_Data.dx = CPI2CM(dx);
                Sensor3_Data.dy = CPI2CM(dy);

                Sensor3_Data.x_dist += Sensor3_Data.dx;
                Sensor3_Data.y_dist += Sensor3_Data.dy;
            }
            // printf("Sensor3 %lld s: dx: %.2f, dy: %.2f, %.2f, %2f\n", Sensor3_Data.timestamps, Sensor3_Data.dx, Sensor3_Data.dy, Sensor3_Data.x_dist, Sensor3_Data.y_dist);

            xSemaphoreGive(spi_bus_mutex);
        }
        delay_ms(10);
    }

}

void start_data_collection_tasks(){
    xTaskCreate(data_collection_task1, "data_collection_task1", 2048, NULL, 2, NULL);
    xTaskCreate(data_collection_task2, "data_collection_task2", 2048, NULL, 2, NULL);
    xTaskCreate(data_collection_task3, "data_collection_task3", 2048, NULL, 2, NULL);
}
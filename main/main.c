#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "PMW3360.h"

void app_main(void)
{

    pmw3360_spi_init(PMW3360_SCLK_GPIO, PMW3360_MISO_GPIO, PMW3360_MOSI_GPIO);
    performStartup(PMW3360_CS0);
    performStartup(PMW3360_CS1);
    performStartup(PMW3360_CS2);

    start_data_collection_tasks();                                        
    printf("Finished Main\n");
}

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include <string.h>

#include "9s12.h"

static const char *TAG = "BDM_Test";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/


void app_main(void)
{
    // gpio_reset_pin(BDM_Pin);
    // /* Set the GPIO as a push/pull output */
    // gpio_set_direction(BDM_Pin, GPIO_MODE_INPUT_OUTPUT_OD);
	// GPIO.out_w1ts  = (1 << BDM_Pin);
    ESP_LOGI(TAG, "Start example BDM test");
	
	// queue = xQueueCreate( 5, sizeof( uint8_t ) );

	// rmt_tx_init();

	xTaskCreate(example_rmt_tx_task, "rmt_tx_task", 4096, NULL, 10, NULL);
	

    // while (1) {
    //     vTaskDelay(1000);
    // }
}

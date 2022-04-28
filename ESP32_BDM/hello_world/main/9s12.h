#ifndef _9S12_H_
#define _9S12_H_

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"

void example_rmt_tx_task(void *arg);

#endif  /* _9S12_H_ */

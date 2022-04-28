#include "9s12.h"

#define RMT_TX_GPIO	18
#define RMT_TX_CHANNEL	RMT_CHANNEL_0

static const char *TAG = "9s12";

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

#define BDM_Pin 18
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1

static uint8_t bdm_data[20] = {0};



static IRAM_ATTR void delay_tc_sample(uint32_t num) {
	for(uint32_t loop = 0; loop < num; loop++)
		__asm__ __volatile__("nop;"); 
}

static IRAM_ATTR void delay_tc(uint32_t num) {
	// uint32_t loop = 0;

	if(num == 2) {
		delay_tc_sample(num);
	}
	else if(num == 4) {
		num *= 2;
		delay_tc_sample(num);
	} 
	else if (num == 10) {
		num *= 3;
		delay_tc_sample(num);
	} 
	else if (num == 12) {
		num *= 4;
		delay_tc_sample(num);
	} 
	else {
		ets_delay_us(100);
	}
}

static bool bdm_sync() {
	//set low for 2ms
	GPIO.out_w1tc = (1 << BDM_Pin);
	ets_delay_us(2000);

	//tongle high
	GPIO.out_w1ts  = (1 << BDM_Pin);

	//wait sync success
	ets_delay_us(100);

	return true;
}


static __inline void bdm_tx_1() {
	//pull low
	GPIO.out_w1tc = (1 << BDM_Pin);

	//delay 4 TC
	delay_tc(4);
	
	//pull high
	GPIO.out_w1ts  = (1 << BDM_Pin);
	//delay 12 TC
	delay_tc(12);
}

static __inline void bdm_tx_0() {
	//pull low
	GPIO.out_w1tc = (1 << BDM_Pin);
	
	//delay 12 TC
	delay_tc(12);
	
	//pull high
	GPIO.out_w1ts  = (1 << BDM_Pin);
	
	//delay 4 TC
	delay_tc(4);
}

static uint8_t bdm_rx() {
	GPIO.out_w1tc = (1 << BDM_Pin);

	gpio_set_direction(BDM_Pin, GPIO_MODE_INPUT);
	
	uint8_t ret = gpio_get_level(BDM_Pin);

	gpio_set_direction(BDM_Pin, GPIO_MODE_OUTPUT);
	GPIO.out_w1ts  = (1 << BDM_Pin);

	return ret;
}

void bdm_send(uint8_t command, uint8_t ack) {
	gpio_set_direction(BDM_Pin, GPIO_MODE_OUTPUT_OD);
	GPIO.out_w1ts  = (1 << BDM_Pin);

	// rmt_tx_reinit();
	// m_rmt_data_prepare(command);
	// rmt_write_items(RMT_TX_CHANNEL, data_send, 8, true);

	for(uint8_t i=0; i<8; i++) {	//shift bit
		if(command & 0x80)
            bdm_tx_1();
        else
            bdm_tx_0();
        command = command << 1;
		delay_tc(4);
	}
	
	if(ack) {
		gpio_set_direction(BDM_Pin, GPIO_MODE_OUTPUT_OD);
		ets_delay_us(25);
	}
}

void bdm_read(uint8_t *byte, uint8_t len) {
	int x, i = 0;
	memset(byte, 0, len);

	gpio_set_direction(BDM_Pin, GPIO_MODE_OUTPUT);
	taskENTER_CRITICAL(&myMutex);
	for(x = 0; x < len; x++) {
		for(i = 0; i <= 7; i++) {
			byte[x] |= (bdm_rx() << (7-i));
		}
	}
	taskEXIT_CRITICAL(&myMutex);
	gpio_set_direction(BDM_Pin, GPIO_MODE_OUTPUT_OD);
}

void bdm_soft_reset() {
	//pull low
	GPIO.out_w1tc = (1 << BDM_Pin);
	ets_delay_us(220000);
	
	//tongle high
	GPIO.out_w1ts  = (1 << BDM_Pin);
	
	//wait sync
	ets_delay_us(110000);
}

void bdm_start() {
	gpio_reset_pin(BDM_Pin);
	gpio_set_direction(BDM_Pin, GPIO_MODE_OUTPUT_OD);

	bdm_soft_reset();

	bdm_sync();
	delay_tc(1024);
	
	bdm_send(0xD5, 1);
	bdm_send(0xE4, 0);
	bdm_send(0xFF, 0);
	bdm_send(0x01, 1);
	bdm_read(bdm_data, 2);
	delay_tc(1024);
}

void bdm_read_address_local(uint16_t address) {
	delay_tc(1024);
	bdm_send(0xD5, 1);	//ACK_ENABLE
	bdm_send(0xE8, 0);	//READ_BD_BYTE
	bdm_send(address >> 8, 0);
	bdm_send(address & 0xFF, 1);
	bdm_read(bdm_data, 2);
}

bool sample_verify(uint16_t data, uint8_t *try_time) {
	static uint16_t old_data = 0;
	static uint8_t check = 0;
	bool ret = false;

	if(data != old_data) {
		old_data = data;
		check = 0;
		ret = false;
	} 
	else {
		check ++;
		if(check > 3) {
			*try_time = check;
			old_data = 0;
			check = 0;
			ret = true;
		}
	}

	return ret;
}

void test_read_eeprom() {
	bdm_start();

	uint16_t address = 0x400;
	uint16_t address_e = 0x440;
	uint16_t data_recv;
	uint8_t try_time;

	while(address <= address_e) {
		do {
			bdm_sync();
			bdm_read_address_local(address);
			data_recv = (bdm_data[0] << 8) | (bdm_data[1]);
			vTaskDelay(1);
		} while (!sample_verify(data_recv, &try_time));
		
		
		ESP_LOGI(TAG, "addr %04X: %02X%02X   try_time %d", address, bdm_data[0], bdm_data[1], try_time);
		address +=2;
	}
}
//////////////////////////////////////////////////////////////////////////


/**
 * @brief RMT Transmit Task
 *
 */
void example_rmt_tx_task(void *arg)
{

    while (1) {
		test_read_eeprom();
        ESP_LOGI(TAG, "Transmission complete");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}


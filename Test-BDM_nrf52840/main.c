
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


//////////////////////////
/* Reference documents MC9S12C128V1 */
/* Table 6-5. Hardware Commands */
#define BDM_ACK_ENABLE		0xD5	//Enable handshake. Issues an ACK pulse after the command is executed

#define BDM_READ_BD_BYTE	0xE4	//Read from memory with standard BDM firmware lookup table in map. Odd address data on low byte; even address data on high byte.
#define BDM_WRITE_BD_BYTE	0xC4	//Write to memory with standard BDM firmware lookup table in map. Oddaddress data on low byte; even address data on high byte.

#define BDM_READ_BYTE		0xE0	//Read from memory with standard BDM firmware lookup table out of map. Odd address data on low byte; even address data on high byte.
#define BDM_WRITE_BYTE		0xC0	//Write to memory with standard BDM firmware lookup table out of map. Odd address data on low byte; even address data on high byte
//////////////////////////


#define BDM_Pin				4


uint8_t bdm_data[20] = {0};
uint32_t volatile tick_data[20] = {0};
volatile bool rx_enable = false;

////////////
#define GPIO_PIN_CNF_OFFSET		0x700
#define GPIO_PIN_OUTSET_OFFSET	0x508
#define GPIO_PIN_OUTCLR_OFFSET	0x50C
#define GPIO_PIN_OUTSTATE_OFFSET 0x504
#define GPIO_PIN_IN 			0x510
#define GPIO_PIN_DIRSET 		0x518
#define GPIO_PIN_DIRCLR 		0x51C

static volatile uint8_t *GPIO = (uint8_t *)0x50000000UL;
#define GPIO_PIN_RESET	0
#define GPIO_PIN_SET	1


/*
*	pin_num:		gpio number
*	pin_level:		0-low; 1-high
*/
static __inline void gpio_pin_write(uint8_t pin_num, uint8_t pin_level) {
	if(pin_level) {
		*((uint32_t *)(GPIO + GPIO_PIN_OUTSET_OFFSET)) = 1<<pin_num;
	} else {
		*((uint32_t *)(GPIO + GPIO_PIN_OUTCLR_OFFSET)) = 1<<pin_num;
	}
}

void gpio_cfg_output(uint8_t pin_num) {
	uint32_t off_set = GPIO_PIN_CNF_OFFSET + pin_num * 4;	//32bit registers
	*((uint32_t *)(GPIO + off_set)) = 	0x01;				//config as out
}

static __inline void gpio_cfg_input(uint32_t pin_num) {
//    *((uint32_t *)(GPIO + GPIO_PIN_DIRCLR)) |= 1UL << pin_num;
	NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin_num);

    reg->PIN_CNF[pin_num] = ((uint32_t)NRF_GPIO_PIN_DIR_INPUT << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)NRF_GPIO_PIN_INPUT_CONNECT << GPIO_PIN_CNF_INPUT_Pos);
}

#pragma GCC push_options
#pragma GCC optimize ("O0")
static __inline void delay_tc(uint32_t num) {

	uint32_t overflow;		//caculate clock cycle
	if(num == 4) {
		overflow = 12;
	} else if(num == 12) {
		overflow = 64;
	} else {
		overflow = num * 4;
	}
	num = 0;
	while(num <= overflow)
		num += 4;			//loop take 4 clock cycle
}
#pragma GCC pop_options

static void bdm_sync(uint8_t sync_enable) {
	nrf_gpio_cfg_output(BDM_Pin);
	
	//set low for 2ms
	gpio_pin_write(BDM_Pin, GPIO_PIN_RESET);
	nrf_delay_ms(2);

	//tongle high
	gpio_pin_write(BDM_Pin, GPIO_PIN_SET);
	
	//wait sync	
	nrf_gpio_cfg_input(BDM_Pin, NRF_GPIO_PIN_NOPULL);
	
	//wait sync success
	nrf_delay_ms(1);
}

#pragma GCC push_options
#pragma GCC optimize ("O0")
static uint8_t bdm_rx() {
        nrf_gpio_cfg_output(BDM_Pin);

	gpio_pin_write(BDM_Pin, GPIO_PIN_RESET);

	gpio_cfg_input(BDM_Pin);
	uint8_t ret = nrf_gpio_pin_read(BDM_Pin);
	delay_tc(12);

	return ret;
}
#pragma GCC pop_options


static __inline void bdm_tx_1() {
	//pull low
	gpio_pin_write(BDM_Pin, GPIO_PIN_RESET);
	
	//delay 4 TC
	delay_tc(2);
	
	//pull high
	gpio_pin_write(BDM_Pin, GPIO_PIN_SET);
	
	//delay 12 TC
	delay_tc(12);
}

static __inline void bdm_tx_0() {
	//pull low
	gpio_pin_write(BDM_Pin, GPIO_PIN_RESET);
	
	//delay 12 TC
	delay_tc(12);
	
	//pull high
	gpio_pin_write(BDM_Pin, GPIO_PIN_SET);
	
	//delay 4 TC
	delay_tc(4);
}


void bdm_send(uint8_t command, uint8_t ack) {
	nrf_gpio_cfg_output(BDM_Pin);
	
	for(uint8_t i=0; i<8; i++) {	//shift bit
		if(command & 0x80)
            bdm_tx_1();
        else
            bdm_tx_0();
        command = command << 1;
	}
	
	if(ack) {
		nrf_delay_us(25);
	}
}

void bdm_read(uint8_t *byte, uint8_t len) {
	memset(byte, 0, len);

	int x, i = 0;
	for(x = 0; x < len; x++) {
		for(i = 0; i <= 7; i++) {
			byte[x] |= (bdm_rx() << (7-i));
		}
	}
}

void bdm_soft_reset() {
	nrf_gpio_cfg_output(BDM_Pin);
	//pull low
	gpio_pin_write(BDM_Pin, GPIO_PIN_RESET);
	nrf_delay_ms(220);
	
	//tongle high
	gpio_pin_write(BDM_Pin, GPIO_PIN_SET);
	
	//wait sync
	nrf_gpio_cfg_input(BDM_Pin, NRF_GPIO_PIN_NOPULL);
	nrf_delay_ms(110);
}

void bdm_start() {
	bdm_soft_reset();
	bdm_sync(1);
	delay_tc(1024);
	
	bdm_send(BDM_ACK_ENABLE, 1);
	
	bdm_send(BDM_READ_BD_BYTE, 0);
	bdm_send(0xFF, 0);
	bdm_send(0x01, 1);
	bdm_read(bdm_data, 2);
	delay_tc(1024);
	
	if(bdm_data[1] != 0) {
		NRF_LOG_INFO("bdm_start ready with code 0x%02X%02X\n", bdm_data[0], bdm_data[1]);
		NRF_LOG_FLUSH();
	} else {
		NRF_LOG_INFO("bdm_start fail \n");
		NRF_LOG_FLUSH();
	}
	nrf_delay_ms(100);
}

void bdm_read_address_local(uint16_t address) {
	delay_tc(1024);
	bdm_send(BDM_ACK_ENABLE, 1);	//ACK_ENABLE
	bdm_send(BDM_READ_BD_BYTE, 0);	//READ_BD_BYTE
	bdm_send(address >> 8, 0);
	bdm_send(address & 0xFF, 1);
	bdm_read(bdm_data, 2);
}

void bdm_read_address_pflash(uint16_t address) {
	delay_tc(1024);
	bdm_send(BDM_READ_BYTE, 0);	//READ_BD_BYTE
	bdm_send(0x80, 0);
	bdm_send(address >> 8, 0);
	bdm_send(address & 0xFF, 1);
	nrf_delay_us(60);
	bdm_read(bdm_data, 2);
}
///////////////////////////////


static void log_init() {
	uint32_t err_code;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void test_read_eeprom() {
	bdm_start();

	static uint16_t address = 0x400;
	static uint16_t address_e = 0x4A0;
	while(address <= address_e) {
		bdm_sync(0);
		bdm_read_address_local(address);
		NRF_LOG_INFO("local addr %04X: %02X%02X", address, bdm_data[0], bdm_data[1]);
		address +=2;
		NRF_LOG_FLUSH();
	}
}

void test_read_pflash() {
	bdm_sync(1);
	nrf_delay_ms(1);
	
	// These steps above are magic
	
	// writing to ram??
	bdm_send(BDM_WRITE_BYTE, 0);
	bdm_send(0x00, 0);
	bdm_send(0x3C, 0);
	bdm_send(0x00, 0);
	bdm_send(0x00, 0);
	nrf_delay_ms(40);
	
	// 0x000B Mode Register (MODE) - 4.3.2.9
	bdm_send(BDM_READ_BYTE, 0);
	bdm_send(0x00, 0);
	bdm_send(0x0B, 0);
	nrf_delay_us(62);
	bdm_read(bdm_data, 2);
	nrf_delay_us(500);
	NRF_LOG_INFO("0x000B value %02X%02X", bdm_data[0], bdm_data[1]);
	
        //  BDM Status Register (BDMSTS) - 6.3.2.1
	bdm_send(0xE4, 0);
	bdm_send(0xFF, 0);
	bdm_send(0x01, 0);
	nrf_delay_us(62);
	bdm_read(bdm_data, 2);
	nrf_delay_us(500);
	NRF_LOG_INFO("0xFF01 value %02X%02X", bdm_data[0], bdm_data[1]);
	
	bdm_send(BDM_READ_BYTE, 0);
	bdm_send(0x01, 0);
	bdm_send(0x01, 0);
	nrf_delay_us(62);
	bdm_read(bdm_data, 2);
	nrf_delay_us(500);
	NRF_LOG_INFO("0x0101 value %02X%02X", bdm_data[0], bdm_data[1]);
	NRF_LOG_FLUSH();
	
	nrf_delay_ms(724);
	bdm_send(BDM_READ_BYTE, 0);
	bdm_send(0x00, 0);
	bdm_send(0x15, 0);
	bdm_send(0x0C, 0);
	bdm_send(0x0C, 0);
	nrf_delay_us(62);
	
	// only tested for range 0x00 - 0x100
	static uint16_t address = 0x00;
	static uint16_t address_e = 0x200;
	while(address <= address_e) {
		bdm_send(0xE0, 0);
		bdm_send(0x80, 0);
		bdm_send(address & 0xFF, 0);
		nrf_delay_us(62);
		bdm_read(bdm_data, 2);
		nrf_delay_ms(50);
                if(bdm_data[0] != 0xFF) {
                  NRF_LOG_INFO("addr %04X: %02X%02X", address, bdm_data[0], bdm_data[1]);
                  NRF_LOG_FLUSH();
                }
		address +=2;
		NRF_LOG_FLUSH();
	}
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
	log_init();
	
	//disable all interrupt
	__disable_irq();
	
	
	nrf_gpio_cfg_output(BDM_Pin);
	gpio_pin_write(BDM_Pin, 1);
	
	nrf_delay_ms(100);
	NRF_LOG_INFO("BDM test started.");
	NRF_LOG_FLUSH();
	
	/////////////////////////
	test_read_eeprom();
        //test_read_pflash();
        /////////////////////////
        

	
    while (true)
    {
		NRF_LOG_FLUSH();
		nrf_delay_ms(100);
    }
}


/** @} */

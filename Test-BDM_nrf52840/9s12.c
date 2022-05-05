#include "9s12.h"
#include "nrf_gpio.h"

#define GPIO_PIN_CNF_OFFSET 0x700
#define GPIO_PIN_OUTSET_OFFSET 0x508
#define GPIO_PIN_OUTCLR_OFFSET 0x50C
#define GPIO_PIN_OUTSTATE_OFFSET 0x504
#define GPIO_PIN_IN 0x510
#define GPIO_PIN_DIRSET 0x518
#define GPIO_PIN_DIRCLR 0x51C

/* ================================== Variables ================================ */
static uint16_t mem_addr = 0x400;
static volatile uint8_t *GPIO = (uint8_t *)0x50000000UL;

/*...............................................................................*/
static __inline void gpio_pin_write(uint8_t pin_num, uint8_t pin_level)
{
  if (pin_level)
    *((uint32_t *)(GPIO + GPIO_PIN_OUTSET_OFFSET)) = 1 << pin_num;
  else
    *((uint32_t *)(GPIO + GPIO_PIN_OUTCLR_OFFSET)) = 1 << pin_num;
}
/*...............................................................................*/
static __inline void gpio_cfg_input(uint32_t pin_num)
{
  NRF_GPIO_Type *reg = nrf_gpio_pin_port_decode(&pin_num);

  reg->PIN_CNF[pin_num] = ((uint32_t)NRF_GPIO_PIN_DIR_INPUT << GPIO_PIN_CNF_DIR_Pos) | ((uint32_t)NRF_GPIO_PIN_INPUT_CONNECT << GPIO_PIN_CNF_INPUT_Pos);
}

/* ============================== 9S12 Interface =============================== */
/**
 * @brief Function for delaying execution for number of microseconds.
 *
 * @note NRF52 has instruction cache and because of that delay is not precise.
 *
 * @param number_of_tc
 */
static __ASM void __INLINE eeprom_9s12_delay_tc(uint32_t volatile number_of_tc)
{
loop
		SUBS    R0, R0, #1
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		NOP
		BNE    loop
		BX     LR
}

/*...............................................................................*/
#pragma push
#pragma O0
static void eeprom_9s12_send_cmd(uint8_t cmd, uint8_t ack)
{

  nrf_gpio_cfg_output(EEPROM_9S12_BDM_PIN);

  for (uint8_t i = 0; i < 8; i++)
  {
    //__disable_irq();
    if (cmd & 0x80)
    {
      gpio_pin_write(EEPROM_9S12_BDM_PIN, 0);
      eeprom_9s12_delay_tc(4);
      gpio_pin_write(EEPROM_9S12_BDM_PIN, 1);
      eeprom_9s12_delay_tc(12);
    }

    else
    {
      gpio_pin_write(EEPROM_9S12_BDM_PIN, 0);
      eeprom_9s12_delay_tc(12);
      gpio_pin_write(EEPROM_9S12_BDM_PIN, 1);
      eeprom_9s12_delay_tc(4);
    }
    //__enable_irq();
    cmd = cmd << 1;
  }

  if (ack) {
    gpio_cfg_input(EEPROM_9S12_BDM_PIN);
    nrf_delay_us(25);
  }

}
#pragma pop
/*...............................................................................*/
#pragma push
#pragma O0
static uint8_t eeprom_9s12_rx(void)
{
  uint8_t byte = 1;
	
  nrf_gpio_cfg_output(EEPROM_9S12_BDM_PIN);
  gpio_pin_write(EEPROM_9S12_BDM_PIN, 0);
	gpio_cfg_input(EEPROM_9S12_BDM_PIN);
	
	eeprom_9s12_delay_tc(4);
	
  byte = nrf_gpio_pin_read(EEPROM_9S12_BDM_PIN);
  eeprom_9s12_delay_tc(12);

  return byte;
}
#pragma pop
/*...............................................................................*/
static void eeprom_9s12_sync(uint8_t enable)
{
  nrf_gpio_cfg_output(EEPROM_9S12_BDM_PIN);

  /* Set low for 1ms */
  gpio_pin_write(EEPROM_9S12_BDM_PIN, 0);
  nrf_delay_ms(1);

  /* Toggle high */
  gpio_pin_write(EEPROM_9S12_BDM_PIN, 1);

  /* Wait for sync */
  gpio_cfg_input(EEPROM_9S12_BDM_PIN);
  nrf_delay_ms(1);
}
/*...............................................................................*/
static void eeprom_9s12_soft_reset(void)
{
  nrf_gpio_cfg_output(EEPROM_9S12_BDM_PIN);

  gpio_pin_write(EEPROM_9S12_BDM_PIN, 0);
  nrf_delay_ms(220);

  gpio_pin_write(EEPROM_9S12_BDM_PIN, 1);

  gpio_cfg_input(EEPROM_9S12_BDM_PIN);
  nrf_delay_ms(110);
}
/*...............................................................................*/
static bool sample_verify(uint16_t data, uint8_t *try_time) {
    static uint16_t old_data = 0;
    static uint8_t check = 0;
    bool ret = false;

    if(data != old_data) {
        old_data = data;
        check = 0;
        ret = false;
    } else {
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

static uint16_t eeprom_9s12_read(uint16_t addr)
{
  uint8_t byte_data[2] = {0};

  eeprom_9s12_delay_tc(1024);
  eeprom_9s12_send_cmd(0xD5, 1);
  eeprom_9s12_send_cmd(0xE8, 0);
  eeprom_9s12_send_cmd(addr >> 8, 0);
  eeprom_9s12_send_cmd(addr & 0xFF, 1);

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j <= 7; j++)
      byte_data[i] |= (eeprom_9s12_rx() << (7 - j));
  }

  nrf_gpio_cfg_output(EEPROM_9S12_BDM_PIN);
  gpio_pin_write(EEPROM_9S12_BDM_PIN, 1);

  return ((byte_data[0] & 0xFF) | (byte_data[1] << 8));
}

/* =============================== 9S12 Tasks ================================== */
bool is_eeprom_9s12_request(uint8_t odo_type)
{
  if (odo_type == EEPROM_9S12_TYPE)
    return true;

  return false;
}
/*...............................................................................*/
static void eeprom_9s12_setup_task(void)
{
  comm_packet c_packet = {CMD_ODO_SETUP_RES, {0}, 1};

  gpio_pin_write(EEPROM_9S12_BDM_PIN, 1);
  nrf_gpio_cfg_output(EEPROM_9S12_BDM_PIN);
  nrf_delay_ms(100);

  eeprom_9s12_soft_reset();
  nrf_delay_ms(1000);
  mem_addr = 0x400;

  c_packet.buffer[0] = CUSTOM_RESERVED_CHAR_UUID & 0xFF;
  c_packet.buffer[1] = (CUSTOM_RESERVED_CHAR_UUID >> 8) & 0xFF;
  c_packet.buffer[2] = 0x01;

  nrf_queue_push(comm_rx_q, (void *)&c_packet);
  setup_object_task_function(NULL);
}
/*...............................................................................*/
static void eeprom_9s12_backup_task(void)
{
  uint16_t read_data;
  uint8_t try_time;

  comm_packet c_packet = {CMD_ODO_READ_DATA_RES, {0}, 0};
	eeprom_9s12_sync(0);
  for (uint8_t i = 0; i < EEPROM_9S12_READ_WRITE_SIZE; i += 2)
  {
    do {
        read_data = eeprom_9s12_read(mem_addr);
    } while (!sample_verify(read_data, &try_time));
    
    c_packet.buffer[i + 2] = read_data & 0xFF;
    c_packet.buffer[i + 3] = (read_data >> 8) & 0xFF;
    mem_addr += 2;
  }

  nrf_delay_ms(1);

  c_packet.buffer[0] = CUSTOM_RESERVED_CHAR_UUID & 0xFF;
  c_packet.buffer[1] = (CUSTOM_RESERVED_CHAR_UUID >> 8) & 0xFF;
  c_packet.buf_len = EEPROM_9S12_READ_WRITE_SIZE;
  nrf_queue_push(comm_rx_q, (void *)&c_packet);
  setup_object_task_function(NULL);
}

/*...............................................................................*/
void eeprom_9s12_perform_task(comm_packet *packet)
{
  switch (packet->cmd & 0x0F)
  {
  case CMD_ODO_SETUP_REQ:
    setup_object_task_function(eeprom_9s12_setup_task);
    break;

  case CMD_ODO_READ_DATA_REQ:
    setup_object_task_function(eeprom_9s12_backup_task);
    break;
  }
}

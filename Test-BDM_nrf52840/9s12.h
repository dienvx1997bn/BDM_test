#ifndef __9S12_H
#define __9S12_H

#include "main.h"
#include "communication.h"

/* ================================ Definition  ================================ */
#define EEPROM_9S12_BDM_PIN             4
#define EEPROM_9S12_READ_WRITE_SIZE     16 /* Could be increase if needed */
#define EEPROM_9S12_TYPE                11

/* ============================= Function Prototype ============================ */
void eeprom_9s12_perform_task(comm_packet *packet);
bool is_eeprom_9s12_request(uint8_t odo_type);

#endif /* __9S12_H */

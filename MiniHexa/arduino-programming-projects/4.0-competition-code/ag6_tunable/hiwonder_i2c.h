#ifndef HIWONDER_I2C_H_
#define HIWONDER_I2C_H_

#include "global.h"
#include "Wire.h"

#ifdef __cplusplus
extern "C" {
#endif

bool wire_write_byte(uint8_t addr, uint8_t val);

bool wire_write_array(uint8_t addr, 
                      uint8_t reg,
                      uint8_t *val,
                      uint16_t len);

int wire_read_array(uint8_t addr, 
                    uint8_t reg, 
                    uint8_t *val, 
                    uint8_t len);
                              
#ifdef __cplusplus
}
#endif

#endif
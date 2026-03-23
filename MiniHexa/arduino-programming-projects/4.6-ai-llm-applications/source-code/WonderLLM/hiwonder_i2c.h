#ifndef HIWONDER_I2C_H_
#define HIWONDER_I2C_H_

#include "global.h"
#include "Wire.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Transmit one byte to slave */
bool wire_write_byte(uint8_t addr, uint8_t val);

/* Write bytes to slave register */
bool wire_write_array(uint8_t addr, 
                      uint8_t reg,
                      uint8_t *val,
                      uint16_t len);

/* Read bytes from slave register */
int wire_read_array(uint8_t addr, 
                    uint8_t reg, 
                    uint8_t *val, 
                    uint8_t len);

/* Write bytes to slave */
bool wireWritemultiByte(uint8_t addr, 
                        uint8_t *val, 
                        unsigned int len);

/* Write bytes to slave */
int wireReadmultiByte(uint8_t addr, 
                      uint8_t *val, 
                      unsigned int len);

#ifdef __cplusplus
}
#endif

#endif
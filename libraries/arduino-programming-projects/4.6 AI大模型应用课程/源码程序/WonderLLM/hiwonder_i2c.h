#ifndef HIWONDER_I2C_H_
#define HIWONDER_I2C_H_

#include "global.h"
#include "Wire.h"

#ifdef __cplusplus
extern "C" {
#endif

/*向从机传输一个字节*/
bool wire_write_byte(uint8_t addr, uint8_t val);

/*向从机寄存器写入若干字节*/
bool wire_write_array(uint8_t addr, 
                      uint8_t reg,
                      uint8_t *val,
                      uint16_t len);

/*在从机寄存器读出若干字节*/
int wire_read_array(uint8_t addr, 
                    uint8_t reg, 
                    uint8_t *val, 
                    uint8_t len);

/*向从机写入若干字节**/
bool wireWritemultiByte(uint8_t addr, 
                        uint8_t *val, 
                        unsigned int len);

/*向从机写入若干字节*/
int wireReadmultiByte(uint8_t addr, 
                      uint8_t *val, 
                      unsigned int len);

#ifdef __cplusplus
}
#endif

#endif
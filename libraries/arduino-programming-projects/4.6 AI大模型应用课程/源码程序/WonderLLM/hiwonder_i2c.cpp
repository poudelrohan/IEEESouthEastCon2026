#include "hiwonder_i2c.h"

bool wire_write_byte(uint8_t addr, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(val);
  if( Wire.endTransmission() != 0 ) {
    return false;
  }
  return true;
}

bool wire_write_array(uint8_t addr, uint8_t reg, uint8_t *val, uint16_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  for(uint16_t i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  if( Wire.endTransmission() != 0 ) {
    return false;
  }
  return true;
}

int wire_read_array(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t len) {
  uint8_t i = 0;  
  
  /* Indicate which register we want to read from */
  if (!wire_write_byte(addr, reg)) {
    return -1;
  }
  Wire.requestFrom(addr, len);
  while (Wire.available()) {
    if (i >= len) {
      return -1;
    }
    val[i] = Wire.read();
    i++;
  }
  return i;
}

//写多个字节（不用寄存器）
bool wireWritemultiByte(uint8_t addr, uint8_t *val, unsigned int len)
{   
  unsigned int sent = 0;
    while(sent < len)
    {
        unsigned int chunk = (len - sent > 512) ? 512 : len - sent;

        Wire.beginTransmission(addr);
        for(unsigned int i = 0; i < chunk; i++)
        {
            Wire.write(val[sent + i]);
        }
        if(Wire.endTransmission() != 0)
            return false;

        sent += chunk;
    }
    return true;
}

//读指定长度字节（不用寄存器）
int wireReadmultiByte(uint8_t addr, uint8_t *val, unsigned int len)
{
    unsigned int readLen = 0;
    while (readLen < len)
    {
        unsigned int chunk = (len - readLen > 512) ? 512 : (len - readLen);

        unsigned int received = Wire.requestFrom(addr, chunk);
        if (received == 0) {
            return false;  // 设备无响应
        }

        for (unsigned int i = 0; i < received && readLen < len; i++)
        {
            val[readLen++] = Wire.read();
        }
    }
    return true; // 返回实际读取的字节数
}

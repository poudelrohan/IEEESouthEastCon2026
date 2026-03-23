#include "iic_data_send.hpp"
#include "Wire.h"

#define I2C_SLAVE_ADDRESS 0x52

static QueueHandle_t xQueueResultI = NULL;
static QueueHandle_t xQueueResultO = NULL;

static const char *TAG = "iic_data_send";
static const int sdaPin = 38;
static const int sclPin = 48;
static const uint32_t i2cFrequency = 100000;

static send_color_data_t send_color_data[4];

static uint8_t rec = 0xFF;
static uint8_t send_data[4] = {0};

static void iic_receive(int len)
{
  while(Wire.available())
  {
    rec = Wire.read();
  }  
}

static void iic_request()
{
  /* Send red blob data */
    if(rec == 0x00) 
    {
        send_data[0] = send_color_data[0].center_x;
        send_data[1] = send_color_data[0].center_y;
        send_data[2] = send_color_data[0].width;
        send_data[3] = send_color_data[0].length;

    }
    /* Send green blob data */
    else if(rec == 0x01)
    {
        send_data[0] = send_color_data[1].center_x;
        send_data[1] = send_color_data[1].center_y;
        send_data[2] = send_color_data[1].width;
        send_data[3] = send_color_data[1].length;

    }
    /* Send blue blob data */
    else if(rec == 0x02)
    {
        send_data[0] = send_color_data[2].center_x;
        send_data[1] = send_color_data[2].center_y;
        send_data[2] = send_color_data[2].width;
        send_data[3] = send_color_data[2].length;

    }
    /* Send purple blob data */
    else if(rec == 0x03)
    {
        send_data[0] = send_color_data[3].center_x;
        send_data[1] = send_color_data[3].center_y;
        send_data[2] = send_color_data[3].width;
        send_data[3] = send_color_data[3].length;
      
    }
    /* Send all blob data */
    else if(rec == 0x04)
    {
        send_data[0] = send_color_data[0].id;
        send_data[1] = send_color_data[1].id;
        send_data[2] = send_color_data[2].id;
        send_data[3] = send_color_data[3].id;      
    }
    /* Pack and send blob data */
    Wire.slaveWrite(send_data, sizeof(send_data));
}

static void task_process_handler(void *arg)
{
  /* I2C initialization */
  Wire.begin((uint8_t)I2C_SLAVE_ADDRESS, sdaPin, sclPin, i2cFrequency);
  /* Register data receive callback */
  Wire.onReceive(iic_receive);
  /* Register data request callback */
  Wire.onRequest(iic_request);

  while (true)
  {

    if (xQueueReceive(xQueueResultI, &send_color_data, portMAX_DELAY))
    {
    //  switch(rec){
    //   case 0x00:
    //     printf("red:%d",rec);
    //     break;
    //   case 0x01:
    //     printf("blue:%d",rec);
    //     break;
    //  }
    }
  }
}


void register_iic_data_send(const QueueHandle_t result_i,
                            const QueueHandle_t result_o)
{
  xQueueResultI = result_i;
  xQueueResultO = result_o;

  xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
}
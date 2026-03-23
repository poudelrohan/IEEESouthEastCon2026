#include "camera_setting.h"
#include "color_detection.hpp"
#include "iic_data_send.hpp"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueIICData = NULL;

void setup() {
  /* Create image transfer queue */
  xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *)); 
  /* Create I2C data transfer queue */
  xQueueIICData = xQueueCreate(2, sizeof(send_color_data_t) * COLOR_NUM);

  /* Register camera processing task */
  register_camera(PIXFORMAT_RGB565, FRAMESIZE_QQVGA, 4, xQueueAIFrame);
  /* Register face detection task */
  register_color_detection(xQueueAIFrame, NULL, xQueueIICData, NULL, true);
  /* Register I2C data transfer task */
  register_iic_data_send(xQueueIICData, NULL);

}

void loop() 
{
}

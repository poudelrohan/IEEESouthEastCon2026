#include "camera_setting.h"
#include "color_detection.hpp"
#include "iic_data_send.hpp"

static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueIICData = NULL;

void setup() {
  /* Create image transfer queue (创建图像传输队列) */
  xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *)); 
  /* Create IIC data transfer queue (创建IIC数据传输队列) */
  xQueueIICData = xQueueCreate(2, sizeof(send_color_data_t *) * COLOR_NUM);

  /* Register camera processing task (注册摄像头处理任务)  */
  register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 4, xQueueAIFrame);
  /* Register color detection task (注册颜色识别任务) */
  register_color_detection(xQueueAIFrame, NULL, xQueueIICData, NULL, true);
  /* Register IIC data transmission task (注册IIC数据传输任务) */
  register_iic_data_send(xQueueIICData, NULL);

}

void loop() 
{
}

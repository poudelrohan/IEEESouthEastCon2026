#include "color_detection.hpp"
#include "esp_log.h"
#include "esp_camera.h"
#include "dl_image.hpp"
#include "fb_gfx.h"
#include "color_detector.hpp"
#include "who_ai_utils.hpp"
#include "camera_setting.h"

using namespace std;
using namespace dl;

static const char *TAG = "color_detection";

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static QueueHandle_t xQueueResult = NULL;

static bool gReturnFB = true;
static int g_max_color_area = 0;
color_data_t color_data[4];



/* 在RGB565图像上绘制边框（支持简单线宽） */
static inline void draw_rect_rgb565(uint16_t *image_ptr, int image_height, int image_width,
                                    int x0, int y0, int x1, int y1, uint16_t color, int thickness)
{
  if (!image_ptr) return;
  if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
  if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
  if (x1 < 0 || y1 < 0 || x0 >= image_width || y0 >= image_height) return;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 >= image_width) x1 = image_width - 1;
  if (y1 >= image_height) y1 = image_height - 1;

  if (thickness < 1) thickness = 1;
  int max_thick = (x1 - x0 + 1);
  if (max_thick > (y1 - y0 + 1)) max_thick = (y1 - y0 + 1);
  if (thickness > max_thick) thickness = max_thick;

  for (int t = 0; t < thickness; ++t)
  {
    int yt_top = y0 + t;
    int yt_bot = y1 - t;
    if (yt_top >= 0 && yt_top < image_height)
    {
      uint16_t *row_top = image_ptr + yt_top * image_width;
      for (int x = x0; x <= x1; ++x) row_top[x] = color;
    }
    if (yt_bot >= 0 && yt_bot < image_height && yt_bot != yt_top)
    {
      uint16_t *row_bot = image_ptr + yt_bot * image_width;
      for (int x = x0; x <= x1; ++x) row_bot[x] = color;
    }

    int xt_left = x0 + t;
    int xt_right = x1 - t;
    if (xt_left >= 0 && xt_left < image_width)
    {
      for (int y = y0; y <= y1; ++y)
      {
        image_ptr[y * image_width + xt_left] = color;
      }
    }
    if (xt_right >= 0 && xt_right < image_width && xt_right != xt_left)
    {
      for (int y = y0; y <= y1; ++y)
      {
        image_ptr[y * image_width + xt_right] = color;
      }
    }
  }
}

/* 颜色阈值 用户可在此处调整 */
vector<color_info_t> std_color_info = {
    {{138, 255, 48, 255, 211, 255}, 64, "red"},
    {{48, 80, 70, 161, 90, 255}, 64, "green"},
    {{94, 105, 28, 138, 79, 255}, 64, "blue"},
    {{125, 155, 70, 255, 90, 255}, 64, "purple"}
};


/* 获取颜色检测的结果 */
static void get_color_detection_result(uint16_t *image_ptr, int image_height, int image_width, vector<color_detect_result_t> &results, uint16_t color)
{
  int g_max_color_column_index = 0;
  /* 寻找同色最大色块 */
  for (int i = 0; i < results.size(); ++i)
  {
    if (results[i].area > g_max_color_area)
    {
      g_max_color_area = results[i].area;
      g_max_color_column_index = i;
    }

    switch (color)
    {
      case COLOR_RED:
        color_data[0].id = 1;
        color_data[0].center_x = (uint8_t)results[g_max_color_column_index].center[0];
        color_data[0].center_y = (uint8_t)results[g_max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[0].width = (uint8_t)(results[g_max_color_column_index].box[2] - results[g_max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[0].length = (uint8_t)(results[g_max_color_column_index].box[3] - results[g_max_color_column_index].box[1]);
        break;

      case COLOR_GREEN:
        color_data[1].id = 2;
        color_data[1].center_x = (uint8_t)results[g_max_color_column_index].center[0];
        color_data[1].center_y = (uint8_t)results[g_max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[1].width = (uint8_t)(results[g_max_color_column_index].box[2] - results[g_max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[1].length = (uint8_t)(results[g_max_color_column_index].box[3] - results[g_max_color_column_index].box[1]);
        break;

      case COLOR_BLUE:
        color_data[2].id = 3;
        color_data[2].center_x = (uint8_t)results[g_max_color_column_index].center[0];
        color_data[2].center_y = (uint8_t)results[g_max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[2].width = (uint8_t)(results[g_max_color_column_index].box[2] - results[g_max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[2].length = (uint8_t)(results[g_max_color_column_index].box[3] - results[g_max_color_column_index].box[1]);
        break;


      case COLOR_PURPLE:
        color_data[3].id = 4;
        color_data[3].center_x = (uint8_t)results[g_max_color_column_index].center[0];
        color_data[3].center_y = (uint8_t)results[g_max_color_column_index].center[1];
        /* right_down_x - left_up_x  */
        color_data[3].width = (uint8_t)(results[g_max_color_column_index].box[2] - results[g_max_color_column_index].box[0]);
        /* right_down_y - left_up_y  */
        color_data[3].length = (uint8_t)(results[g_max_color_column_index].box[3] - results[g_max_color_column_index].box[1]);
        break;

      default:
        break;
    }
  }
  /* 本函数不绘制，仅更新 color_data */
}

static void task_process_handler(void *arg)
{
  camera_fb_t *frame = NULL;
  ColorDetector detector;
  /* 注册颜色信息 */
  for (int i = 0; i < std_color_info.size(); ++i)
  {
    detector.register_color(std_color_info[i].color_thresh, std_color_info[i].area_thresh, std_color_info[i].name);
  }
  vector<uint16_t> draw_colors = {
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_PURPLE,
  };
  int draw_colors_num = draw_colors.size();
  while (true)
  {
    if (xQueueReceive(xQueueFrameI, &frame, portMAX_DELAY))
    {
      std::vector<std::vector<color_detect_result_t>> &results = detector.detect((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
      for(int i = 0; i < COLOR_NUM; ++i)
      {
        if(results[i].size() == 0)
        {
          color_data[i].id = 0;
          color_data[i].center_x = 0;
          color_data[i].center_y = 0;
          color_data[i].width = 0;
          color_data[i].length = 0;
        }
      }
      
      for (int i = 0; i < results.size(); ++i)
      {
        get_color_detection_result((uint16_t *)frame->buf, (int)frame->height, (int)frame->width, results[i], draw_colors[i % draw_colors_num]);
      }

      /* 仅绘制所有颜色中面积最大的一个色块 */
      int best_color_idx = -1;
      int best_result_idx = -1;
      int best_area = 0;
      for (int i = 0; i < results.size(); ++i)
      {
        if (results[i].empty()) continue;
        int local_best = 0;
        int local_idx = 0;
        for (int j = 0; j < results[i].size(); ++j)
        {
          if (results[i][j].area > local_best)
          {
            local_best = results[i][j].area;
            local_idx = j;
          }
        }
        if (local_best > best_area)
        {
          best_area = local_best;
          best_color_idx = i;
          best_result_idx = local_idx;
        }
      }
      if (best_color_idx >= 0 && best_result_idx >= 0)
      {
        int x0 = (int)results[best_color_idx][best_result_idx].box[0];
        int y0 = (int)results[best_color_idx][best_result_idx].box[1];
        int x1 = (int)results[best_color_idx][best_result_idx].box[2];
        int y1 = (int)results[best_color_idx][best_result_idx].box[3];
        uint16_t color = draw_colors[best_color_idx % draw_colors_num];
        draw_rect_rgb565((uint16_t *)frame->buf, (int)frame->height, (int)frame->width, x0, y0, x1, y1, color, 2);
      }

      // 显示处理后的画面到TFT
      tft_show_rgb565((const uint16_t *)frame->buf, frame->width, frame->height);
    }
    if (xQueueFrameO)
    {
      xQueueSend(xQueueFrameO, &frame, portMAX_DELAY);
    }
    else if (gReturnFB)
    {
      esp_camera_fb_return(frame);
    }
    else
    {
      free(frame);
    }
    if (xQueueResult)
    {
      xQueueSend(xQueueResult, &color_data, portMAX_DELAY);             
    }          
  }
}

void register_color_detection(const QueueHandle_t frame_i,
                                   const QueueHandle_t event,
                                   const QueueHandle_t result,
                                   const QueueHandle_t frame_o,
                                   const bool camera_fb_return)
{
  xQueueFrameI = frame_i;
  xQueueFrameO = frame_o;
  xQueueEvent = event;
  xQueueResult = result;
  gReturnFB = camera_fb_return;

  xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);

}
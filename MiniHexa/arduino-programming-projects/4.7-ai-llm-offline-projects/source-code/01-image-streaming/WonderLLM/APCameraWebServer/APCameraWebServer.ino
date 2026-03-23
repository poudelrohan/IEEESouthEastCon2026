#include "esp_camera.h"
#include <WiFi.h>
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
// #define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"
#include "lib/adafruit/Adafruit_GFX.h"
#include "lib/adafruit/Adafruit_ST7789.h"
#include <SPI.h>

#define TFT_CS   2
#define TFT_DC   1
#define TFT_RST -1   // Set to -1 if tied to 3.3V
#define TFT_BL  14

void tft_show_rgb565(const uint16_t *rgb565_buf, int width, int height);
void startCameraServer();

static Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "HW_ESP32S3CAM";
const char* password = "";
IPAddress local_ip(192, 168, 5, 1);       // Custom IP address
IPAddress gateway(192, 168, 1, 1);        // Gateway address
IPAddress subnet(255, 255, 255, 0);       // Subnet mask

void setup() 
{
  Serial.begin(115200);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 15000000;
  /* @param frame_size   One of
   *                     - FRAMESIZE_96X96,    // 96x96
   *                     - FRAMESIZE_QQVGA,    // 160x120
   *                     - FRAMESIZE_QCIF,     // 176x144
   *                     - FRAMESIZE_HQVGA,    // 240x176
   *                     - FRAMESIZE_240X240,  // 240x240
   *                     - FRAMESIZE_QVGA,     // 320x240
   *                     - FRAMESIZE_CIF,      // 400x296
   *                     - FRAMESIZE_HVGA,     // 480x320
   *                     - FRAMESIZE_VGA,      // 640x480
   *                     - FRAMESIZE_SVGA,     // 800x600
   *                     - FRAMESIZE_XGA,      // 1024x768
   *                     - FRAMESIZE_HD,       // 1280x720
   *                     - FRAMESIZE_SXGA,     // 1280x1024
   *                     - FRAMESIZE_UXGA,     // 1600x1200
   *                     - FRAMESIZE_FHD,      // 1920x1080
   *                     - FRAMESIZE_P_HD,     //  720x1280
   *                     - FRAMESIZE_P_3MP,    //  864x1536
   *                     - FRAMESIZE_QXGA,     // 2048x1536
   *                     - FRAMESIZE_QHD,      // 2560x1440
   *                     - FRAMESIZE_WQXGA,    // 2560x1600
   *                     - FRAMESIZE_P_FHD,    // 1080x1920
   *                     - FRAMESIZE_QSXGA,    // 2560x1920
   */
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_RGB565; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    // config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
#if defined(CAMERA_MODEL_ESP32S3_EYE)
  // s->set_vflip(s, 1);
#endif
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  // Initialize SPI (must specify SCK, MOSI pins)
  SPI.begin(21, -1, 47);  // SCK=21, MISO=-1 (unused), MOSI=47

  // Initialize display (resolution must match)
  tft.init(240, 320);
  tft.setRotation(3);  // Rotate to 320x240 coordinate system
  // Increase SPI clock for faster refresh rate
  tft.setSPISpeed(80000000);
  tft.fillScreen(ST77XX_BLACK);  // Fill with black background

  xTaskCreatePinnedToCore(
        tft_task,     
        "TFT_Task",   
        4096,         
        NULL,         
        1,            
        NULL,         
        1             
    );

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, 6, false, 4); // SSID, password, channel 6, not hidden, max 4 connections
  // WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  // Configure IP address
  if (!WiFi.softAPConfig(local_ip, gateway, subnet)) {
    Serial.println("Failed to configure IP");
  }
  Serial.println("WiFi AP Started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP()); // Print AP IP address


  startCameraServer();
  // Serial.print(WiFi.softAPIP());
  // Serial.println("' to connect");
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}

void tft_show_rgb565(const uint16_t *rgb565_buf, int width, int height)
{
    if (!rgb565_buf || width <= 0 || height <= 0) return;

    int pixel_count = width * height;
    static uint16_t *work_buf = nullptr;
    static int work_capacity = 0;
    if (work_capacity < pixel_count) {
        if (work_buf) free(work_buf);
        work_buf = (uint16_t *)malloc(pixel_count * sizeof(uint16_t));
        work_capacity = pixel_count;
    }

    // Byte swap
    for (int i = 0; i < pixel_count; i++) {
        uint16_t v = rgb565_buf[i];
        work_buf[i] = (v << 8) | (v >> 8);
    }

    int screen_w = tft.width();
    int screen_h = tft.height();

    // Special case: camera 240x320, screen 320x240
    if (width == 240 && height == 320 && screen_w == 320 && screen_h == 240) {
        static uint16_t *rotated = nullptr;
        static int rotated_capacity = 0;
        int rotated_pixels = screen_w * screen_h;
        if (rotated_capacity < rotated_pixels) {
            if (rotated) free(rotated);
            rotated = (uint16_t *)malloc(rotated_pixels * sizeof(uint16_t));
            rotated_capacity = rotated_pixels;
        }

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int dst_x = y;
                int dst_y = width - 1 - x;
                // Horizontal mirror modification
                dst_y = screen_w - 1 - dst_y;
                rotated[dst_y * screen_w + dst_x] = work_buf[y * width + x];
            }
        }
        tft.drawRGBBitmap(0, 0, rotated, screen_w, screen_h);
        return;
    }

    // Same resolution, draw fullscreen
    if (width == screen_w && height == screen_h) {
        static uint16_t *mirror_buf = nullptr;
        static int mirror_capacity = 0;
        if (mirror_capacity < pixel_count) {
            if (mirror_buf) free(mirror_buf);
            mirror_buf = (uint16_t *)malloc(pixel_count * sizeof(uint16_t));
            mirror_capacity = pixel_count;
        }
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                mirror_buf[y * width + x] = work_buf[y * width + (width - 1 - x)]; // Horizontal mirror
            }
        }
        tft.drawRGBBitmap(0, 0, mirror_buf, width, height);
        return;
    }

    // Other scale/crop branch (Cover mode)
    static uint16_t *scaled = nullptr;
    static int scaled_capacity = 0;
    int scaled_pixels = screen_w * screen_h;
    if (scaled_capacity < scaled_pixels) {
        if (scaled) free(scaled);
        scaled = (uint16_t *)malloc(scaled_pixels * sizeof(uint16_t));
        scaled_capacity = scaled_pixels;
    }

    bool scale_by_width = ((int64_t)screen_w * height >= (int64_t)screen_h * width);
    if (scale_by_width) {
        int visible_src_h = (int)((int64_t)screen_h * width / screen_w);
        if (visible_src_h > height) visible_src_h = height;
        int src_y_offset = (height - visible_src_h) / 2;

        static int *y_map = nullptr;
        static int y_map_cap = 0;
        if (y_map_cap < screen_h) {
            if (y_map) free(y_map);
            y_map = (int *)malloc(screen_h * sizeof(int));
            y_map_cap = screen_h;
        }
        for (int dy = 0; dy < screen_h; dy++) {
            y_map[dy] = src_y_offset + (int)((int64_t)dy * visible_src_h / screen_h);
        }

        static int *x_map = nullptr;
        static int x_map_cap = 0;
        if (x_map_cap < screen_w) {
            if (x_map) free(x_map);
            x_map = (int *)malloc(screen_w * sizeof(int));
            x_map_cap = screen_w;
        }
        for (int dx = 0; dx < screen_w; dx++) {
            x_map[dx] = (int)((int64_t)dx * width / screen_w);
            // Horizontal mirror modification
            x_map[dx] = width - 1 - x_map[dx];
        }

        for (int dy = 0; dy < screen_h; dy++) {
            int src_y = y_map[dy];
            const uint16_t *src_row = &work_buf[src_y * width];
            uint16_t *dst_row = &scaled[dy * screen_w];
            for (int dx = 0; dx < screen_w; dx++) {
                int src_x = x_map[dx];
                dst_row[dx] = src_row[src_x];
            }
        }
    } else {
        int visible_src_w = (int)((int64_t)screen_w * height / screen_h);
        if (visible_src_w > width) visible_src_w = width;
        int src_x_offset = (width - visible_src_w) / 2;

        static int *y_map = nullptr;
        static int y_map_cap = 0;
        if (y_map_cap < screen_h) {
            if (y_map) free(y_map);
            y_map = (int *)malloc(screen_h * sizeof(int));
            y_map_cap = screen_h;
        }
        for (int dy = 0; dy < screen_h; dy++) {
            y_map[dy] = (int)((int64_t)dy * height / screen_h);
        }

        static int *x_map = nullptr;
        static int x_map_cap = 0;
        if (x_map_cap < screen_w) {
            if (x_map) free(x_map);
            x_map = (int *)malloc(screen_w * sizeof(int));
            x_map_cap = screen_w;
        }
        for (int dx = 0; dx < screen_w; dx++) {
            x_map[dx] = src_x_offset + (int)((int64_t)dx * visible_src_w / screen_w);
            // Horizontal mirror modification
            x_map[dx] = width - 1 - x_map[dx];
        }

        for (int dy = 0; dy < screen_h; dy++) {
            int src_y = y_map[dy];
            const uint16_t *src_row = &work_buf[src_y * width];
            uint16_t *dst_row = &scaled[dy * screen_w];
            for (int dx = 0; dx < screen_w; dx++) {
                int src_x = x_map[dx];
                dst_row[dx] = src_row[src_x];
            }
        }
    }

    tft.drawRGBBitmap(0, 0, scaled, screen_w, screen_h);
}


void tft_task(void *pvParameters) {
    camera_fb_t *fb = nullptr;

    while (true) {
        // Get camera frame
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // Display frame
        tft_show_rgb565((const uint16_t *)fb->buf, fb->width, fb->height);

        // Release camera frame
        esp_camera_fb_return(fb);

        vTaskDelay(33 / portTICK_PERIOD_MS);
    }
}
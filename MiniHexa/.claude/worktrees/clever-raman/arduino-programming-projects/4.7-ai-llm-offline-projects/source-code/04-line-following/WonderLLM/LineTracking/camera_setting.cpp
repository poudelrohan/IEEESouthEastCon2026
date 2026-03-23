#include "camera_setting.h"
#include "esp_log.h"
#include "esp_system.h"
#include "lib/adafruit/Adafruit_GFX.h"
#include "lib/adafruit/Adafruit_ST7789.h"
#include <SPI.h>

#define TFT_CS   2
#define TFT_DC   1
#define TFT_RST -1   // Set to -1 if tied to 3.3V
#define TFT_BL  14

static Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

static const char *TAG = "camera";

static QueueHandle_t xQueueFrameO = NULL;

static void task_process_handler(void *arg)
{
    uint32_t frame_count = 0;
    uint32_t dropped_frames = 0;
    
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
        {
            frame_count++;
            // Try to send frame to queue, release if queue full
            if (xQueueSend(xQueueFrameO, &frame, 0) != pdTRUE)
            {
                // Queue full, release current frame
                esp_camera_fb_return(frame);
                dropped_frames++;
                if (dropped_frames % 100 == 0) {
                    ESP_LOGW(TAG, "Dropped %d frames, total frames: %d", dropped_frames, frame_count);
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "Failed to get camera frame");
        }
        // Add small delay to avoid high CPU usage
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void register_camera(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count,
                     const QueueHandle_t frame_o)

{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

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
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.frame_size = frame_size;
    config.pixel_format = pixel_fromat; // for streaming
    config.grab_mode = CAMERA_GRAB_LATEST;  // Always get latest frame
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 16;
    config.fb_count = fb_count;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1); //flip it back    
    } else if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);
    } else if (s->id.PID == GC032A_PID) {
        s->set_vflip(s, 1);
    }
    
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  //up the blightness just a bit
        s->set_saturation(s, -2); //lower the saturation
    }
    
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

    xQueueFrameO = frame_o;
    xTaskCreatePinnedToCore(task_process_handler, TAG, 3 * 1024, NULL, 5, NULL, 1);
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

#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dl_image.hpp"
#include "dl_image_jpeg.hpp"
#include "hand_detect.hpp"
#include "hand_gesture_recognition.hpp"

#include "camera_pins.h"
#include <list>
#include <cstring>

static const char *TAG = "hand_gesture_realtime";

/* =========================================================
 * Center crop + resize to 224x224 (nearest)
 * Input: RGB888
 * ========================================================= */
static dl::image::img_t crop_resize_224(const dl::image::img_t &src)
{
    const int target = 224;
    int crop = (src.width < src.height) ? src.width : src.height;
    int x0 = (src.width - crop) / 2;
    int y0 = (src.height - crop) / 2;

    dl::image::img_t dst = {};
    dst.width = target;
    dst.height = target;
    dst.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

    dst.data = heap_caps_malloc(target * target * 3,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!dst.data) {
        dst.data = heap_caps_malloc(target * target * 3,
                                    MALLOC_CAP_8BIT);
    }
    if (!dst.data) {
        ESP_LOGE(TAG, "Failed to alloc resized image");
        return dst;
    }

    uint8_t *src_u8 = (uint8_t *)src.data;
    uint8_t *dst_u8 = (uint8_t *)dst.data;

    for (int y = 0; y < target; ++y) {
        int sy = y0 + (y * crop) / target;
        for (int x = 0; x < target; ++x) {
            int sx = x0 + (x * crop) / target;
            int sidx = (sy * src.width + sx) * 3;
            int didx = (y * target + x) * 3;
            dst_u8[didx + 0] = src_u8[sidx + 0];
            dst_u8[didx + 1] = src_u8[sidx + 1];
            dst_u8[didx + 2] = src_u8[sidx + 2];
        }
    }
    return dst;
}

/* ================= Camera init ================= */
static esp_err_t init_camera()
{
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    config.frame_size   = FRAMESIZE_240X240;
    config.jpeg_quality = 12;
    config.fb_count     = 1;

    return esp_camera_init(&config);
}

/* ================= Main ================= */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Init camera...");
    ESP_ERROR_CHECK(init_camera());
    ESP_LOGI(TAG, "Camera ready");

    /* 丢弃前几帧，稳定 AE/AWB */
    for (int i = 0; i < 5; ++i) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    HandDetect *hand_detect = new HandDetect();
    HandGestureRecognizer *gesture =
        new HandGestureRecognizer(
            HandGestureCls::MOBILENETV2_0_5_S8_V1);

    int frame = 0;

    while (true) {
        frame++;

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        dl::image::jpeg_img_t jpeg = {
            .data = fb->buf,
            .data_len = fb->len
        };

        auto rgb = dl::image::sw_decode_jpeg(
            jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);

        esp_camera_fb_return(fb);

        if (!rgb.data || rgb.width <= 0 || rgb.height <= 0) {
            ESP_LOGE(TAG, "JPEG decode failed");
            if (rgb.data) heap_caps_free(rgb.data);
            continue;
        }

        auto img224 = crop_resize_224(rgb);
        heap_caps_free(rgb.data);

        if (!img224.data) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* ========== Hand Detect ========== */
        auto det_results = hand_detect->run(img224);

        if (det_results.empty()) {
            ESP_LOGW(TAG, "[%d] No hand detected", frame);
            heap_caps_free(img224.data);
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        /* ========== Gesture Classify ========== */
        auto cls_results = gesture->recognize(img224, det_results);

        for (auto &res : cls_results) {
            ESP_LOGI(TAG,
                     "[%d] Gesture: %s  score=%.2f",
                     frame,
                     res.cat_name,
                     res.score);
        }

        heap_caps_free(img224.data);

        vTaskDelay(pdMS_TO_TICKS(2000));  // ~1 FPS，稳定观察
    }
}

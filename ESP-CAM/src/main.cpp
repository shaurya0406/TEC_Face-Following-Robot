#include <Arduino.h>

#include "camera_index.h"
#include "esp_camera.h"
#include "fb_gfx.h"
#include "fd_forward.h"

const int numReadings = 5;
int readings[numReadings];      
int readIndex = 0;             
int total = 0;                  
int smoothed_face_height = 30;                
int face_distance;
int new_setting_turn_speed;
int new_setting_car_speed;
unsigned long check_time;
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 60; // 80 default
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();


void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 2, 14);

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
 
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif


  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes)
{
  int x, y, w, h, i, half_width, half_height;
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {


    x = ((int)boxes->box[i].box_p[0]);
    w = (int)boxes->box[i].box_p[2] - x + 1;
    half_width = w / 2;
    int face_center_pan = x + half_width; 

    y = (int)boxes->box[i].box_p[1];
    h = (int)boxes->box[i].box_p[3] - y + 1;
    half_height = h / 2;
    int face_center_tilt = y + half_height; 

    Serial.println(h);

    total = total - readings[readIndex];
   
    readings[readIndex] = h;
 
    total = total + readings[readIndex];
 
    readIndex = readIndex + 1;

    if (readIndex >= numReadings) {
   
      readIndex = 0;
    }


    smoothed_face_height = total / numReadings;

    int eq_top = 3.6 * 200 * 240; 
    int eq_bottom = smoothed_face_height * 2.7; 
    int face_distance = eq_top / eq_bottom;
    Serial.println(face_distance);

    Serial2.print('<'); 
    Serial2.print(face_center_pan);
    Serial2.print(',');
    Serial2.print(face_center_tilt);
    Serial2.print(','); 
    Serial2.print(face_distance);
    Serial2.println('>'); 

    new_setting_turn_speed = map(face_center_pan, 0, 320, 40, -40); 
    new_setting_car_speed = map(face_distance, 10, 1200, -20, 20); 
  }
}

void loop()
{
 
  camera_fb_t * fb = NULL;
  dl_matrix3du_t *image_matrix = NULL;
  fb = esp_camera_fb_get();

  image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
  box_array_t *net_boxes = NULL;
  net_boxes = face_detect(image_matrix, &mtmn_config);

  if (net_boxes) {
    draw_face_boxes(image_matrix, net_boxes);
    free(net_boxes->score);
    free(net_boxes->box);
    free(net_boxes->landmark);
    free(net_boxes);
  }
  esp_camera_fb_return(fb);
  fb = NULL;
  dl_matrix3du_free(image_matrix);

    if (millis() - check_time > 100) {

      if (new_setting_turn_speed > 0) {
        new_setting_turn_speed -= 1;
      }
       if (new_setting_turn_speed < 0) {
        new_setting_turn_speed += 1;
      }
      if (new_setting_car_speed > 0) {
        new_setting_car_speed -= 1;
      }
      if (new_setting_car_speed < 0) {
        new_setting_car_speed += 1;
      }    
      check_time = millis();
    }

}
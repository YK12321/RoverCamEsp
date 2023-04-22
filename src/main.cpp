#include <Arduino.h>
#include <esp_camera.h>

// Function to convert an RGB image to grayscale
uint8_t* toGrayscale(uint8_t* src, size_t width, size_t height) {
  uint8_t* dst = (uint8_t*) malloc(width * height);
  for (size_t i = 0; i < width * height; i++) {
    uint8_t r = src[i * 3];
    uint8_t g = src[i * 3 + 1];
    uint8_t b = src[i * 3 + 2];
    dst[i] = (r * 0.3) + (g * 0.59) + (b * 0.11);
  }
  return dst;
}
int computeOtsuThreshold(uint8_t* , size_t );
int detectObstacles(uint8_t* , size_t , size_t , int );

void setup() {
  Serial.begin(9600);


#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_GPIO_NUM      33

  // Initialize the camera module
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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  // Capture a frame from the camera
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to capture frame");
    return;
  }

  // Convert the captured frame to grayscale
  uint8_t *gray = toGrayscale(fb->buf, fb->width, fb->height);

  // Compute the threshold using Otsu's method
  int threshold = computeOtsuThreshold(gray, fb->width * fb->height);

  // Detect obstacles using thresholding and edge detection
  int num_obstacles = detectObstacles(gray, fb->width, fb->height, threshold);

  //Inform other MCU about the status of obstacles
  if(num_obstacles <= 10)
  {
    Serial.println("No Obstacles");
  }
  else if(num_obstacles> 10)
  {
    Serial.println("Obstacle Detected");
  }
  //This isn't supposed to happen but just in case
  else
  {
    Serial.println("Error");
  }
  // Free the frame buffer and grayscale image buffer
  esp_camera_fb_return(fb);
  free(gray);
}

// Function to compute the threshold using Otsu's method
int computeOtsuThreshold(uint8_t* src, size_t size) {
  int hist[256] = {0};
  double sum = 0, sumB = 0, wB = 0, wF = 0, maxVar = 0, threshold = 0;

  // Compute the histogram of the image
  for (size_t i = 0; i < size; i++) {
    hist[src[i]]++;
    sum += src[i];
  }

  // Compute the threshold
  for (size_t i = 0; i < 256; i++) {
    wB += hist[i];
    if (wB == 0) continue;
    wF = size - wB;
    if (wF == 0) break;
    sumB += i * hist[i];
    double mB = sumB / wB;
    double mF = (sum - sumB) / wF;
    double varBetween = wB * wF * pow(mB - mF, 2);
    if (varBetween > maxVar) {
      maxVar = varBetween;
      threshold = 2*i;
    }
  }

  return threshold;
}

// Function to detect obstacles based on thresholding and edge detection
int detectObstacles(uint8_t* src, size_t width, size_t height, int threshold) {
  int num_obstacles = 0;
  for (size_t y = 1; y < height - 1; y++) {
    for (size_t x = 1; x < width - 1; x++) {
      uint8_t p = src[y * width + x];
      uint8_t p_left = src[y * width + (x - 1)];
      uint8_t p_right = src[y * width + (x + 1)];
      uint8_t p_up = src[(y - 1) * width + x];
      uint8_t p_down = src[(y + 1) * width + x];

      // Check for color similarity
      if (abs(p - threshold) < 10) {
        // Check for edge detection 
        int gx = abs(p_left - p_right) + abs(p_up - p_down);
        int gy = abs(p_up - p_down) + abs(p_left - p_right);
        int grad = sqrt(gx * gx + gy * gy);
        if (grad > 100) {
          // Increment the obstacle count
          num_obstacles++;
        }
      }
    }
  }
  return num_obstacles;
}
//function to send image data over bluetooth (using built in bluetooth module)
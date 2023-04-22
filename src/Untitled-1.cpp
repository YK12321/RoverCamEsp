#include <ArduinoBLE.h>
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
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


void setup() {
  Serial.begin(9600);
  
  BLEDevice::init("ESP32 BLE Server");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();
  
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE");
    return;
  }

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

  // Create a BLE service with a characteristic for the image data
  BLEService imageService("180D");
  BLECharacteristic imageDataCharacteristic("2A37", BLERead | BLENotify, fb->len);
  imageService.addCharacteristic(imageDataCharacteristic);
  BLE.addService(imageService);
  BLE.advertise();

  // Transfer the image data over BLE
  for (size_t i = 0; i < fb->len; i += BLE_MTU_SIZE) {
    size_t chunkSize = min(BLE_MTU_SIZE, fb->len - i);
    imageDataCharacteristic.writeValue(gray + i, chunkSize);
    delay(10);  // Wait for the BLE stack to process the data
  }

  // Free the frame buffer and grayscale image buffer
  esp_camera_fb_return(fb);
free(gray);

// Return the threshold value
return threshold;
}

// Function to detect obstacles using thresholding and edge detection
int detectObstacles(uint8_t* src, size_t width, size_t height, int threshold) {
// Create a buffer to hold the binary image
uint8_t* binary = (uint8_t*) malloc(width * height);
if (!binary) {
Serial.println("Failed to allocate memory for binary image");
return -1;
}

// Threshold the image to create a binary image
for (size_t i = 0; i < width * height; i++) {
binary[i] = (src[i] >= threshold) ? 255 : 0;
}

// Perform edge detection on the binary image
uint8_t* edges = (uint8_t*) malloc(width * height);
if (!edges) {
Serial.println("Failed to allocate memory for edges image");
free(binary);
return -1;
}
for (size_t i = 1; i < height - 1; i++) {
for (size_t j = 1; j < width - 1; j++) {
int gx = src[(i - 1) * width + (j + 1)] - src[(i - 1) * width + (j - 1)]
+ 2 * src[i * width + (j + 1)] - 2 * src[i * width + (j - 1)]
+ src[(i + 1) * width + (j + 1)] - src[(i + 1) * width + (j - 1)];
int gy = src[(i + 1) * width + (j - 1)] - src[(i - 1) * width + (j - 1)]
+ 2 * src[(i + 1) * width + j] - 2 * src[(i - 1) * width + j]
+ src[(i + 1) * width + (j + 1)] - src[(i - 1) * width + (j + 1)];
int mag = sqrt(gx * gx + gy * gy);
edges[i * width + j] = (mag > 128) ? 255 : 0;
}
}

// Count the number of obstacles
int num_obstacles = 0;
for (size_t i = 0; i < width * height; i++) {
if (edges[i] == 255 && binary[i] == 0) {
num_obstacles++;
}
}

// Free the binary and edges image buffers
free(binary);
free(edges);

// Return the number of obstacles
return num_obstacles;
}

// Function to send an image over Bluetooth using the built-in BLE module for ESP32
void sendImageOverBluetooth(camera_fb_t * fb) {
  // Connect to the central device
  BLEDevice central = BLE.central();
  
  // If we have a connection
  if (central)
  {
    // Send the image data
    int packetSize = 512;
    uint8_t packet[packetSize];
    int remainingBytes = fb->len;
    int offset = 0;
    while (remainingBytes > 0)
    {
      int packetBytes = min(packetSize, remainingBytes);
      memcpy(packet, fb->buf + offset, packetBytes);
      imageCharacteristic.writeValue(packet, packetBytes);
      offset += packetBytes;
      remainingBytes -= packetBytes;
    }
  }
}
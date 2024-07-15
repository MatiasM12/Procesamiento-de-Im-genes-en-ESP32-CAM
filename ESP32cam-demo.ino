#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // Read and write from flash memory
#include "TJpg_Decoder.h"

#define EEPROM_SIZE 1

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

uint8_t *image;

bool convertRGB565toRGB888(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      int index = ((y + j) * IMG_WIDTH + (x + i)) * 3;
      uint16_t pixel = bitmap[j * w + i];
      uint8_t r = (pixel & 0xF800) >> 8;
      uint8_t g = (pixel & 0x07E0) >> 3;
      uint8_t b = (pixel & 0x001F) << 3;
      image[index] = r;
      image[index + 1] = g;
      image[index + 2] = b;
    }
  }
  return true;
}

void Sobel(uint8_t *imageIn, uint8_t *imageOut, int width, int height) {
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      int gx = 0;
      int gy = 0;
      int pixelIndex = y * width + x;

      int Gx[3][3] = {
        { -1, 0, 1 },
        { -2, 0, 2 },
        { -1, 0, 1 }
      };

      int Gy[3][3] = {
        { -1, -2, -1 },
        { 0, 0, 0 },
        { 1, 2, 1 }
      };

      for (int ky = -1; ky <= 1; ky++) {
        for (int kx = -1; kx <= 1; kx++) {
          int sampleIndex = (y + ky) * width + (x + kx);
          int pixelValue = imageIn[sampleIndex];
          gx += Gx[ky + 1][kx + 1] * pixelValue;
          gy += Gy[ky + 1][kx + 1] * pixelValue;
        }
      }

      int gradient = sqrt(gx * gx + gy * gy);
      gradient = constrain(gradient, 0, 255);
      imageOut[pixelIndex] = gradient;
    }
  }
}

void Threshold(uint8_t *grayImage, uint8_t *thresholdImage, int width, int height, uint8_t threshold) {
  for (int i = 0; i < width * height; i++) {
    thresholdImage[i] = (grayImage[i] > threshold) ? 255 : 0;
  }
}

void detectContours(uint8_t *image, int width, int height, int &minX, int &minY, int &maxX, int &maxY) {
  minX = width;
  minY = height;
  maxX = 0;
  maxY = 0;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (image[y * width + x] > 0) {
        if (x < minX) minX = x;
        if (y < minY) minY = y;
        if (x > maxX) maxX = x;
        if (y > maxY) maxY = y;
      }
    }
  }
}

void drawBoundingBox(uint8_t *image, int width, int height, int minX, int minY, int maxX, int maxY) {
  for (int x = minX; x <= maxX; x++) {
    image[minY * width * 3 + x * 3] = 255;
    image[minY * width * 3 + x * 3 + 1] = 0;
    image[minY * width * 3 + x * 3 + 2] = 0;

    image[maxY * width * 3 + x * 3] = 255;
    image[maxY * width * 3 + x * 3 + 1] = 0;
    image[maxY * width * 3 + x * 3 + 2] = 0;
  }
  for (int y = minY; y <= maxY; y++) {
    image[y * width * 3 + minX * 3] = 255;
    image[y * width * 3 + minX * 3 + 1] = 0;
    image[y * width * 3 + minX * 3 + 2] = 0;

    image[y * width * 3 + maxX * 3] = 255;
    image[y * width * 3 + maxX * 3 + 1] = 0;
    image[y * width * 3 + maxX * 3 + 2] = 0;
  }
}

int pictureNumber = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  if (psramFound()) {
    Serial.println("PSRAM found and initialized.");
    image = (uint8_t *)ps_malloc(IMG_WIDTH * IMG_HEIGHT * 3);
  } else {
    Serial.println("No PSRAM found.");
    image = (uint8_t *)malloc(IMG_WIDTH * IMG_HEIGHT * 3);
  }

  if (!image) {
    Serial.println("Failed to allocate memory for image buffer");
    return;
  }

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
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(convertRGB565toRGB888);

  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }

  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);

  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (fb) {
    if (TJpgDec.drawJpg(0, 0, fb->buf, fb->len) != 0) {
      Serial.println("Decoding failed!");
    } else {

      uint8_t *grayImage = (uint8_t *)malloc(IMG_WIDTH * IMG_HEIGHT);
      
      for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
        grayImage[i] = 0.3 * image[i * 3] + 0.59 * image[i * 3 + 1] + 0.11 * image[i * 3 + 2];
      }

      uint8_t *sobelImage = (uint8_t *)malloc(IMG_WIDTH * IMG_HEIGHT);
      Sobel(grayImage, sobelImage, IMG_WIDTH, IMG_HEIGHT);
      free(grayImage);

      uint8_t *thresholdImage = (uint8_t *)malloc(IMG_WIDTH * IMG_HEIGHT);
      Threshold(sobelImage, thresholdImage, IMG_WIDTH, IMG_HEIGHT, 100);
      free(sobelImage);

      int minX, minY, maxX, maxY;
      detectContours(thresholdImage, IMG_WIDTH, IMG_HEIGHT, minX, minY, maxX, maxY);

      drawBoundingBox(image, IMG_WIDTH, IMG_HEIGHT, minX, minY, maxX, maxY);

      String path = "/picture" + String(pictureNumber) + ".jpg";

      // Convertir la imagen filtrada a RGB888 para guardar como BMP
      for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
        uint8_t filteredPixel = thresholdImage[i];
        image[i * 3] = filteredPixel;
        image[i * 3 + 1] = filteredPixel;
        image[i * 3 + 2] = filteredPixel;
      }
      free(thresholdImage);

      // Guardar la imagen en la tarjeta SD en formato BMP
      fs::FS &fs = SD_MMC;
      Serial.printf("Picture file name: %s\n", path.c_str());

      File file = fs.open(path.c_str(), FILE_WRITE);
      if (!file) {
        Serial.println("Failed to open file in writing mode");
      } else {
        // Escribir la cabecera BMP
        uint32_t fileSize = IMG_WIDTH * IMG_HEIGHT * 3 + 54;  // Tamaño del archivo en bytes
        uint32_t imgSize = IMG_WIDTH * IMG_HEIGHT * 3;        // Tamaño de la imagen en bytes
        uint32_t offset = 54;                                 // Desplazamiento hasta los datos de la imagen

        uint8_t bmpHeader[54] = {
          0x42, 0x4D,                                                                                 // BMP signature
          fileSize & 0xFF, (fileSize >> 8) & 0xFF, (fileSize >> 16) & 0xFF, (fileSize >> 24) & 0xFF,  // File size
          0x00, 0x00, 0x00, 0x00,                                                                     // Reserved
          0x36, 0x00, 0x00, 0x00,                                                                     // Offset to image data
          0x28, 0x00, 0x00, 0x00,                                                                     // Header size
          IMG_WIDTH & 0xFF, (IMG_WIDTH >> 8) & 0xFF, 0x00, 0x00,                                      // Image width
          IMG_HEIGHT & 0xFF, (IMG_HEIGHT >> 8) & 0xFF, 0x00, 0x00,                                    // Image height
          0x01, 0x00,                                                                                 // Number of color planes
          0x18, 0x00,                                                                                 // Bits per pixel (24 bits)
          0x00, 0x00, 0x00, 0x00,                                                                     // Compression method
          imgSize & 0xFF, (imgSize >> 8) & 0xFF, (imgSize >> 16) & 0xFF, (imgSize >> 24) & 0xFF,      // Image size
          0x13, 0x0B, 0x00, 0x00,                                                                     // Horizontal resolution (2835 pixels/m)
          0x13, 0x0B, 0x00, 0x00,                                                                     // Vertical resolution (2835 pixels/m)
          0x00, 0x00, 0x00, 0x00,                                                                     // Number of colors in the palette
          0x00, 0x00, 0x00, 0x00                                                                      // Number of important colors
        };

        file.write(bmpHeader, sizeof(bmpHeader));
        file.write(image, IMG_WIDTH * IMG_HEIGHT * 3);
        Serial.printf("Saved file to path: %s\n", path.c_str());
        EEPROM.write(0, pictureNumber);
        EEPROM.commit();
      }
      file.close();
    }
     esp_camera_fb_return(fb);
  }
 

  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);

  delay(2000);
  Serial.println("Going to sleep now");
  delay(2000);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop() {
}

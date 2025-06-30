#include <esp_camera.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// Kamera Pin-Definitionen
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

// Display pins
#define TFT_CS     12   
#define TFT_RST    13  
#define TFT_DC     2
#define TFT_SCLK   14
#define TFT_MOSI   15

// Button Control
#define BUTTON_PIN    4

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Timer und Button State
unsigned long lastActivation = 0;
const unsigned long ACTIVE_TIME = 5UL * 60 * 1000; // 5 Minuten in Millisekunden
bool isActive = false;
bool systemReady = false;
unsigned long lastFrameTime = 0;
uint8_t frameCounter = 0;
bool lastButtonState = LOW;
bool buttonPressed = false;

// FPS Calculation
unsigned long lastFPSUpdate = 0;
uint8_t fpsCounter = 0;
uint8_t currentFPS = 0;

// Error Handling
uint8_t cameraErrors = 0;
uint8_t bufferErrors = 0;
unsigned long lastErrorTime = 0;
bool errorRecoveryMode = false;

// Buffer
uint16_t* displayBuffer = nullptr;

// Software Rotation
void flipImageVertical(uint8_t* image, int width, int height) {
  uint8_t* temp_row = (uint8_t*)malloc(width);
  if (!temp_row) return;
  
  for (int y = 0; y < height / 2; y++) {
    memcpy(temp_row, &image[y * width], width);
    memcpy(&image[y * width], &image[(height - 1 - y) * width], width);
    memcpy(&image[(height - 1 - y) * width], temp_row, width);
  }
  
  free(temp_row);
}

void convertGrayscaleToRGB565(uint8_t* grayscale, uint16_t* rgb565, int width, int height) {
  for (int i = 0; i < width * height; i++) {
    uint8_t gray = grayscale[i];
    
    // Nachtsicht-Look mit grünem Tint
    uint8_t r = gray >> 4;        // Weniger Rot
    uint8_t g = gray >> 2;        // Mehr Grün (6-bit)  
    uint8_t b = gray >> 4;        // Weniger Blau
    
    rgb565[i] = (r << 11) | (g << 5) | b;
  }
}

// Error Display Function
void displayError(const char* errorMsg, uint8_t errorCode) {
  tft.fillScreen(ST77XX_RED);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 20);
  tft.print("FEHLER:");
  tft.setCursor(10, 35);
  tft.print(errorMsg);
  tft.setCursor(10, 50);
  tft.print("Code: ");
  tft.print(errorCode);
  tft.setCursor(10, 70);
  tft.print("Neustart...");
  Serial.print("ERROR: ");
  Serial.print(errorMsg);
  Serial.print(" Code: ");
  Serial.println(errorCode);
}

// Camera Reinitialization
bool reinitializeCamera() {
  Serial.println("Reinitialisiere Kamera...");
  
  // Kamera deinitialize
  esp_camera_deinit();
  delay(500);
  
  // Kamera neu konfigurieren
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 5;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    return false;
  }

  // Sensor neu konfigurieren
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 2);
    s->set_saturation(s, -1);
    s->set_gainceiling(s, (gainceiling_t)6);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_gain_ctrl(s, 1);
  }
  
  delay(500);
  return true;
}

// Buffer Recovery
bool reallocateBuffer() {
  if (displayBuffer) {
    free(displayBuffer);
    displayBuffer = nullptr;
  }
  
  delay(100);
  displayBuffer = (uint16_t*)malloc(160 * 120 * sizeof(uint16_t));
  return (displayBuffer != nullptr);
}

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("=== ESP32 Nachtsichtgerät ===");
  // Display Init
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(5, 10);
  tft.print("NACHTSICHTGERAET");
  tft.setCursor(5, 25);
  tft.print("Initialisierung...");
  // Buffer Allocation
  displayBuffer = (uint16_t*)malloc(160 * 120 * sizeof(uint16_t));
  if (!displayBuffer) {
    tft.fillScreen(ST77XX_RED);
    tft.setCursor(10, 40);
    tft.print("SPEICHER FEHLER!");
    while(1) delay(1000);
  }
  // Kamera Konfiguration
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 5;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    tft.fillScreen(ST77XX_RED);
    tft.setCursor(10, 40);
    tft.print("KAMERA FEHLER!");
    while (1) delay(1000);
  }

  // Sensor Optimierung für Nachtsicht
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 2);
    s->set_saturation(s, -1);
    s->set_gainceiling(s, (gainceiling_t)6);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 0);
    s->set_ae_level(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_hmirror(s,0);
  }
  delay(1000);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(100);
  systemReady = true;

  // Bereit-Anzeige
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10, 30);
  tft.print("NACHTSICHTGERAET");
  tft.setCursor(30, 50);
  tft.print("BEREIT!");
  tft.setCursor(25, 70);
  tft.print("Taste druecken"); 
}


void loop() {
  if (!systemReady) {
    delay(50);
    return;
  }

  unsigned long now = millis();
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Button Debouncing und Toggle-Logik
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    buttonPressed = true;
    delay(50); // Entprellung
  }
  lastButtonState = currentButtonState;

  if (buttonPressed) {
    buttonPressed = false;
    lastActivation = now;
    if (!isActive) {
      isActive = true;
      frameCounter = 0;
      lastFrameTime = now;
      
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(5, 5);
      tft.print("NACHTSICHT AKTIV");
    }
  }

  if (isActive) {
    // Framerate Optimierung - nur alle 50ms verarbeiten
    if (now - lastFrameTime >= 50) {
      lastFrameTime = now;
      
      camera_fb_t *fb = esp_camera_fb_get();
      
      if (fb && fb->format == PIXFORMAT_GRAYSCALE && 
          fb->width == 160 && fb->height == 120 && fb->len > 0) {
        
        // Buffer Check
        if (!displayBuffer) {
          displayError("Buffer NULL", ++bufferErrors);
          if (!reallocateBuffer()) {
            displayError("Buffer Alloc", bufferErrors);
            delay(2000);
            ESP.restart();
          }
          delay(1000);
          esp_camera_fb_return(fb);
          return;
        }
        
        // Software Rotation
        flipImageVertical(fb->buf, 160, 120);
        
        // Konvertierung und Anzeige
        convertGrayscaleToRGB565(fb->buf, displayBuffer, 160, 120);
        tft.drawRGBBitmap(0, 20, displayBuffer, 160, 120);
        
        frameCounter++;
        fpsCounter++;
        
        // Reset error counter on success
        cameraErrors = 0;
        errorRecoveryMode = false;
        
        // FPS Update alle 1000ms
        if (now - lastFPSUpdate >= 5000) {
          currentFPS = fpsCounter;
          fpsCounter = 0;
          lastFPSUpdate = now;
          
          // FPS anzeigen (rechts oben)
          tft.fillRect(120, 5, 38, 10, ST77XX_BLACK);
          tft.setCursor(120, 5);
          tft.setTextColor(ST77XX_GREEN);
          tft.print("FPS:");
          tft.print(currentFPS);
        }
      } else {
        // Camera Error Handling
        cameraErrors++;
        lastErrorTime = now;
        
        if (cameraErrors > 5 && !errorRecoveryMode) {
          errorRecoveryMode = true;
          displayError("Kamera Fehler", cameraErrors);
          
          if (!reinitializeCamera()) {
            displayError("Kamera Init", cameraErrors);
            delay(3000);
            ESP.restart();
          } else {
            cameraErrors = 0;
            tft.fillScreen(ST77XX_BLACK);
            tft.setTextColor(ST77XX_GREEN);
            tft.setCursor(10, 40);
            tft.print("Kamera OK");
            delay(1000);
          }
        }
      }
      
      if (fb) esp_camera_fb_return(fb);
    }

    // Timeout Check
    if (now - lastActivation > ACTIVE_TIME) {
      isActive = false;
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(20, 40);
      tft.print("NIGHT VISION");
      tft.setCursor(30, 60);
      tft.print("STANDBY");
      tft.setCursor(25, 80);
      tft.print("Taste druecken");
    }
  }

  delay(10); // Reduziert CPU-Last
}

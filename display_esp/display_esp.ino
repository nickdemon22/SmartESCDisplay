#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_SDA 8
#define I2C_SCL 9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

typedef struct __attribute__((packed)) {
  float speed_kmh;
  float voltage;
  uint8_t battery_pct;
  uint8_t vesc_ok;
} TelemetryPacket;

TelemetryPacket rxData = {0.0f, 0.0f, 0, 0};
volatile unsigned long last_packet_time = 0;

// Callback приема ESP-NOW (совместим с ESP32 Core 2.x и 3.x)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&rxData, data, sizeof(TelemetryPacket));
    last_packet_time = millis();
  }
}

void renderDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // 1. Потеря сигнала (только если нет пакетов более 2.5 секунд)
  if (millis() - last_packet_time > 2500) {
    display.setTextSize(1);
    display.setCursor(25, 28);
    display.print("SEARCHING...");
    display.display();
    return;
  }

  // 2. Радио на связи, но контроллер выключен
  if (!rxData.vesc_ok) {
    display.setTextSize(1);
    display.setCursor(15, 20);
    display.print("VESC DISCONNECTED");
    display.setCursor(20, 36);
    display.print("Waiting VESC...");
    display.display();
    return;
  }

  // 3. ШТАТНЫЙ ДЭШБОРД
  // Скорость крупными цифрами по центру
  int spd = (int)round(rxData.speed_kmh);
  char spdStr[8];
  snprintf(spdStr, sizeof(spdStr), "%d", spd);

  display.setTextSize(4);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(spdStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 6);
  display.print(spdStr);

  // Метка км/ч по центру под цифрами
  display.setTextSize(1);
  display.setCursor(53, 41);
  display.print("km/h");

  // Тонкая линия-разделитель
  display.drawFastHLine(0, 51, SCREEN_WIDTH, SSD1306_WHITE);

  // Снизу слева: Напряжение (Voltage)
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.printf("%.1fV", rxData.voltage);

  // Снизу справа: Процент АКБ
  char batStr[8];
  snprintf(batStr, sizeof(batStr), "%d%%", rxData.battery_pct);
  display.getTextBounds(batStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w, 55);
  display.print(batStr);

  display.display();
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Автоопределение адреса I2C (0x3C или 0x3D)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 28);
  display.print("SMARTESC READY");
  display.display();

  // Фиксация на канале 1 без сна радиомодуля
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  static unsigned long last_draw = 0;
  // Отрисовка с частотой 16 кадров/сек (раз в 60 мс)
  if (millis() - last_draw >= 60) {
    last_draw = millis();
    renderDisplay();
  }
}
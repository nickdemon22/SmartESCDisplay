#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define VESC_RX_PIN 20 // К TX контроллера
#define VESC_TX_PIN 21 // К RX контроллера

// Коэффициент ERPM -> км/ч для колес 8.5" (30 магнитов / 15 пар полюсов)
// Если стоят покрышки 10" — замените на 0.00319f
const float ERPM_TO_KMH = 0.002714f;

// Границы 10S батареи M365 для расчета процента
const float BATTERY_MIN_V = 31.0f; // 0%
const float BATTERY_MAX_V = 41.5f; // 100%

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Пакет данных для передачи по ESP-NOW
typedef struct __attribute__((packed)) {
  float speed_kmh;
  float voltage;
  uint8_t battery_pct;
  uint8_t vesc_ok;
} TelemetryPacket;

TelemetryPacket txPacket = {0.0f, 0.0f, 0, 0};

// Запрос COMM_GET_VALUES для VESC (CRC16 = 0x4084)
const uint8_t VESC_POLL_CMD[] = { 0x02, 0x01, 0x04, 0x40, 0x84, 0x03 };
uint8_t broadcastMac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;

const int BLE_CHUNK_SIZE = 256; 
uint8_t ble_tx_buf[BLE_CHUNK_SIZE];
size_t ble_tx_len = 0;
unsigned long last_uart_rx_time = 0;
unsigned long last_poll_time = 0;
unsigned long last_send_time = 0;
unsigned long last_valid_vesc_time = 0;

uint8_t vesc_buf[512];
int vesc_idx = 0;

// Парсинг телеметрии VESC
void parseVescPayload(uint8_t *data, int len) {
  if (len < 30 || data[0] != 0x04) return; // COMM_GET_VALUES

  // Байт 23..26: ERPM мотора (int32 big-endian)
  int32_t erpm = ((int32_t)data[23] << 24) | ((int32_t)data[24] << 16) | 
                 ((int32_t)data[25] << 8)  | ((int32_t)data[26]);

  // Байт 27..28: Напряжение АКБ (int16 big-endian, в 0.1V)
  int16_t vin_raw = ((int16_t)data[27] << 8) | (int16_t)data[28];
  float voltage = vin_raw / 10.0f;

  float speed = abs(erpm) * ERPM_TO_KMH;
  if (speed < 0.4f) speed = 0.0f;

  int pct = (int)((voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0f);
  if (pct > 100) pct = 100;
  if (pct < 0) pct = 0;

  txPacket.speed_kmh = speed;
  txPacket.voltage = voltage;
  txPacket.battery_pct = (uint8_t)pct;
  txPacket.vesc_ok = 1;
  last_valid_vesc_time = millis();
}

// Потоковый разбор пакетов VESC UART
void processVescStream(uint8_t b) {
  if (vesc_idx == 0) {
    if (b == 0x02 || b == 0x03) vesc_buf[vesc_idx++] = b;
  } else if (vesc_idx == 1) {
    vesc_buf[vesc_idx++] = b;
  } else {
    vesc_buf[vesc_idx++] = b;
    int payload_len = 0;
    int header_offset = 0;

    if (vesc_buf[0] == 0x02) {
      payload_len = vesc_buf[1];
      header_offset = 2;
    } else {
      if (vesc_idx < 3) return;
      payload_len = ((int)vesc_buf[1] << 8) | vesc_buf[2];
      header_offset = 3;
    }

    int full_len = header_offset + payload_len + 3;
    if (vesc_idx >= full_len) {
      if (vesc_buf[full_len - 1] == 0x03) {
        parseVescPayload(&vesc_buf[header_offset], payload_len);
      }
      vesc_idx = 0;
    }
    if (vesc_idx >= sizeof(vesc_buf)) vesc_idx = 0;
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      BLEDevice::startAdvertising();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial1.write((const uint8_t*)rxValue.c_str(), rxValue.length());
      }
    }
};

void setup() {
  // Настройка UART без Serial.begin, чтобы не занимать пины 20 и 21
  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);

  // 1. Фиксация Wi-Fi канала 1 и максимальная мощность
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // 2. Инициализация BLE для приложения VESC Tool на телефоне
  BLEDevice::init("M365_SmartESC");
  BLEDevice::setMTU(517); 

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  // Опрос VESC каждые 100 мс (если телефон не подключен)
  if (!deviceConnected) {
    if (millis() - last_poll_time >= 100) {
      last_poll_time = millis();
      Serial1.write(VESC_POLL_CMD, sizeof(VESC_POLL_CMD));
    }
  }

  // Прием входящих байт от контроллера
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    processVescStream(b);

    if (deviceConnected && ble_tx_len < BLE_CHUNK_SIZE) {
      ble_tx_buf[ble_tx_len++] = b;
      last_uart_rx_time = millis();
    }
  }

  if (millis() - last_valid_vesc_time > 1000) {
    txPacket.vesc_ok = 0;
  }

  // Частая отправка по ESP-NOW (каждые 40 мс = 25 раз/сек) для стабильности
  if (millis() - last_send_time >= 40) {
    last_send_time = millis();
    esp_now_send(broadcastMac, (uint8_t *)&txPacket, sizeof(txPacket));
  }

  // Трансляция данных в телефон по BLE
  if (deviceConnected && ble_tx_len > 0) {
    if (ble_tx_len >= BLE_CHUNK_SIZE || (millis() - last_uart_rx_time > 3)) {
      pTxCharacteristic->setValue(ble_tx_buf, ble_tx_len);
      pTxCharacteristic->notify();
      ble_tx_len = 0;
      delay(2);
    }
  }

  delay(1);
}
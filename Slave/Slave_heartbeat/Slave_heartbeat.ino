
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define PIN_PPG           35     // ADC1
#define SAMPLE_PERIOD_MS  5      // 200 Hz sampling untuk deteksi puncak
#define SEND_PERIOD_MS    100    // 10 Hz kirim; Master menganggap link putus setelah 1000 ms
#define BEAT_TIMEOUT_MS   2500   // tidak ada detak valid selama ini -> BPM = 0

// GANTI dengan MAC Master: lihat Serial Monitor Master, baris "LOG,master mac=..."
uint8_t masterAddress[] = {0x8C, 0xAA, 0xB5, 0x37, 0x9E, 0x64};

// Harus identik dengan struct di Master
struct DataPacket {
  int bpm;
};

DataPacket heartData;
esp_now_peer_info_t peerInfo;

unsigned long lastBeatTime = 0;
int  signalThreshold = 2200;     // masih tetap (belum adaptive) - sesuaikan dengan sensor
bool pulseState = false;
int  currentBpm = 0;

volatile uint32_t g_sendFail = 0;
uint32_t sendCount = 0;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) g_sendFail++;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PPG, INPUT);

  WiFi.mode(WIFI_STA);
  Serial.printf("slave mac=%s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp-now init failed");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;          // 0 = ikut channel saat ini (harus sama dengan Master)
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("add peer failed");
  }
}

void loop() {
  static TickType_t lastWake = xTaskGetTickCount();
  static uint32_t lastSendMs = 0, lastLogMs = 0;

  int rawPPG = analogRead(PIN_PPG);
  unsigned long now = millis();

  if (rawPPG < 300 || rawPPG > 4000) {
    currentBpm = 0;   // tidak ada kontak jari
  } else {
    if (rawPPG > signalThreshold && !pulseState) {
      pulseState = true;
      unsigned long ibi = now - lastBeatTime;
      lastBeatTime = now;
      if (ibi > 300 && ibi < 1500) currentBpm = 60000 / ibi;
    }
    if (rawPPG < (signalThreshold - 100)) pulseState = false;

    // Tidak ada detak valid terlalu lama -> jangan tampilkan BPM lama
    if (now - lastBeatTime > BEAT_TIMEOUT_MS) currentBpm = 0;
  }

  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    heartData.bpm = currentBpm;
    esp_now_send(masterAddress, (uint8_t *)&heartData, sizeof(heartData));
    sendCount++;
  }

  if (now - lastLogMs >= 1000) {   // debug lokal (Serial Slave tidak ke Unity)
    lastLogMs = now;
    Serial.printf("raw=%d bpm=%d sent=%lu fail=%lu\n", rawPPG, currentBpm,
                  (unsigned long)sendCount, (unsigned long)g_sendFail);
  }

  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
}

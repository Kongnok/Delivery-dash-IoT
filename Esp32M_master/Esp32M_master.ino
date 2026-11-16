// ESP32 Master - Smart Bicycle (Tema 1)
// Scope saat ini: struktur firmware, task LCD I2C, task RFID RC522, format Serial ke Unity.
// Core: Arduino-ESP32 v3.x (signature callback ESP-NOW v3).
// Library: LiquidCrystal_I2C, MFRC522 (miguelbalboa).
//
// ==========================================================================
// FORMAT DATA KE UNITY (Serial 115200 baud, 1 paket = 1 baris, diakhiri '\n')
// Dipilih lewat SERIAL_FORMAT di bagian konfigurasi. Isi data sama, bentuk beda.
//
//  Versi 1 - COMPACT (key=value, ringan, parsing: Split(',') lalu Split('='))
//    D,v=1,pot=2048,bpm=72,link=1,rfid=04A1B2C3,session=1,rpm=-1,weight=-1
//
//  Versi 2 - JSON (satu objek per baris; siap dipakai sebagai payload MQTT)
//    {"v":1,"pot":2048,"bpm":72,"link":1,"rfid":"04A1B2C3","session":1,"rpm":-1,"weight":-1}
//
//  Arti field (sama di kedua versi):
//    v       versi skema paket (naikkan jika struktur berubah)
//    pot     0-4095  ADC potensiometer (kemudi)
//    bpm     detak jantung; 0 = tidak ada data valid (jari lepas / link Slave putus)
//    link    1 = paket ESP-NOW dari Slave diterima dalam LINK_TIMEOUT_MS terakhir
//    rfid    UID hex selama RFID_HOLD_MS setelah tap, selain itu "0"
//    session 1 = sesi aktif (tap pertama = mulai, tap berikutnya = akhiri)
//    rpm     -1 = BELUM DIIMPLEMENTASI (encoder FC-03); nanti: putaran/menit
//    weight  -1 = BELUM DIIMPLEMENTASI (weight sensor); satuan belum ditetapkan
//  Nilai -1 = "sensor belum ada / tidak tersedia", bukan hasil ukur. Sengaja bukan
//  0 (0 rpm = pedal berhenti) dan bukan null (JsonUtility Unity tidak mendukung
//  nullable). Unity harus mengabaikan field bernilai -1.
//
//  Baris log diawali "LOG," -> Unity hanya memproses baris yang diawali
//  "D," (versi 1) atau "{" (versi 2).
//
// ---- CATATAN RENCANA MQTT ----------------------------------------------------
//  * Builder paket (buildCompact / buildJson) terpisah dari pengiriman Serial.
//    Saat MQTT ditambahkan, panggil buildJson() lalu publish buffer yang sama.
//  * Jangan publish MQTT pada 20 Hz seperti Serial; cukup ~1-2 Hz (atau saat
//    session/rfid berubah) agar broker & jaringan tidak terbebani.
//  * RISIKO: MQTT mengharuskan Master terhubung ke Wi-Fi AP. Saat itu radio Master
//    pindah ke channel AP, sedangkan ESP-NOW butuh Master dan Slave di channel yang
//    sama -> komunikasi Slave bisa putus. Slave harus diset ke channel AP yang sama,
//    atau MQTT dipindah ke ESP32 gateway terpisah. Perlu diverifikasi saat integrasi.
//  * Gunakan field "v" untuk menjaga kompatibilitas backend/Unity bila skema berubah.
// ==========================================================================

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <MFRC522.h>

// ==========================================
// 1. KONFIGURASI
// ==========================================
#define PIN_POTENTIO     34     // ADC1 (aman dipakai bersamaan dengan WiFi/ESP-NOW)
#define PIN_I2C_SDA      21
#define PIN_I2C_SCL      22
#define LCD_ADDR         0x27   // Ganti ke 0x3F jika LCD tidak menyala

// RC522 (SPI dengan pin custom)
#define PIN_RFID_RST     23
#define PIN_RFID_MOSI    16
#define PIN_RFID_MISO    17
#define PIN_RFID_SCK     25
#define PIN_RFID_SS      26     // pin "SDA" pada modul RC522

#define SERIAL_FORMAT_COMPACT 1
#define SERIAL_FORMAT_JSON    2
#define SERIAL_FORMAT    SERIAL_FORMAT_JSON   // pilih versi paket ke Unity
#define PACKET_VERSION   1

// MQTT: 0 = mati (tanpa Wi-Fi/MQTT). 1 = aktif.
// Jika 1: butuh library PubSubClient (Nick O'Leary) dan isi kredensial di bawah.
// TESTING ONLY: broker publik HiveMQ tanpa autentikasi -> siapa pun bisa membaca/menulis topik.
// Jangan kirim data pribadi sungguhan (UID kartu, berat badan) ke broker ini.
#define ENABLE_MQTT      1
#define WIFI_SSID        "ISI_SSID"
#define WIFI_PASS        "ISI_PASSWORD"
#define MQTT_HOST        "broker.hivemq.com"
#define MQTT_PORT        1883
#define MQTT_TOPIC_DATA   "itenas/d11/deliverydash/telemetry"   // prefix unik: broker publik dipakai bersama
#define MQTT_TOPIC_STATUS "itenas/d11/deliverydash/status"      // "online"/"offline" (LWT, retained)
#define MQTT_PUBLISH_MS  1000                 // 1 Hz; Serial ke Unity tetap 20 Hz
#define NOT_AVAILABLE    (-1)   // sensor belum diimplementasi / tidak tersedia

#define SERIAL_PERIOD_MS 50     // 20 Hz
#define LCD_PERIOD_MS    250
#define RFID_POLL_MS     50
#define TAP_DEBOUNCE_MS  1000   // abaikan tap ulang dalam rentang ini
#define RFID_HOLD_MS     2000   // status "TAPPED" bertahan selama ini setelah tap
#define LINK_TIMEOUT_MS  1000   // Slave dianggap putus bila tidak ada paket selama ini

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
MFRC522 mfrc522(PIN_RFID_SS, PIN_RFID_RST);

// ==========================================
// 2. DATA ESP-NOW & SHARED STATE
// ==========================================
// Harus identik dengan struct di Slave
struct DataPacket {
  int bpm;
};

struct SharedState {
  int      bpm;
  int      potRaw;
  uint32_t lastRecvMs;    // 0 = belum pernah menerima paket
  char     uid[21];       // UID kartu terakhir (hex)
  uint32_t lastTapMs;     // 0 = belum pernah tap
  bool     session;       // true = sesi aktif (di-toggle oleh tap RFID)
  int      rpm;           // NOT_AVAILABLE sampai encoder diimplementasi (isi dari task encoder)
  int      weight;        // NOT_AVAILABLE sampai weight sensor diimplementasi
};

struct Snapshot {
  int  bpm;
  int  pot;
  bool link;
  bool tapped;
  bool session;
  int  rpm;
  int  weight;
  char uid[21];
};

enum RfidState { RFID_STARTING = 0, RFID_OK = 1, RFID_ERROR = 2 };

static SharedState g_state = {0, 0, 0, {0}, 0, false, NOT_AVAILABLE, NOT_AVAILABLE};
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int g_rfidState = RFID_STARTING;

// Snapshot state + status turunan. Bila link putus, bpm dipaksa 0 (tidak stale).
static void readState(Snapshot &o) {
  SharedState s;
  portENTER_CRITICAL(&stateMux);
  s = g_state;
  portEXIT_CRITICAL(&stateMux);

  uint32_t now = millis();
  o.link   = (s.lastRecvMs != 0) && ((uint32_t)(now - s.lastRecvMs) < LINK_TIMEOUT_MS);
  o.bpm    = o.link ? s.bpm : 0;
  o.pot    = s.potRaw;
  o.tapped = (s.lastTapMs != 0) && ((uint32_t)(now - s.lastTapMs) < RFID_HOLD_MS);
  o.session = s.session;
  o.rpm     = s.rpm;
  o.weight  = s.weight;
  strncpy(o.uid, s.uid, sizeof(o.uid));
  o.uid[sizeof(o.uid) - 1] = '\0';
}

// ==========================================
// 3. CALLBACK ESP-NOW (Core v3.x)
// ==========================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(DataPacket)) return;   // abaikan paket tidak valid

  DataPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  portENTER_CRITICAL(&stateMux);
  g_state.bpm        = pkt.bpm;
  g_state.lastRecvMs = millis();
  portEXIT_CRITICAL(&stateMux);
}

// ==========================================
// 4. TASK RFID (FreeRTOS, Core 1)
// ==========================================
void TaskRFID(void *pvParameters) {
  (void) pvParameters;

  SPI.begin(PIN_RFID_SCK, PIN_RFID_MISO, PIN_RFID_MOSI, PIN_RFID_SS);
  mfrc522.PCD_Init();

  // Cek modul terpasang: VersionReg 0x00/0xFF = tidak ada respon (kabel/power salah)
  byte ver = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);
  if (ver == 0x00 || ver == 0xFF) {
    g_rfidState = RFID_ERROR;
    Serial.println("LOG,rfid reader not found");
    vTaskDelete(NULL);
  }
  g_rfidState = RFID_OK;
  Serial.printf("LOG,rfid ready ver=0x%02X\n", ver);

  uint32_t lastToggleMs = 0;
  for (;;) {
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      char hex[21] = {0};
      for (byte i = 0; i < mfrc522.uid.size && i < 10; i++) {
        snprintf(hex + 2 * i, 3, "%02X", mfrc522.uid.uidByte[i]);
      }

      uint32_t now = millis();
      bool newSession;
      portENTER_CRITICAL(&stateMux);
      memcpy(g_state.uid, hex, sizeof(hex));
      g_state.lastTapMs = now ? now : 1;   // 0 dicadangkan = "belum pernah"
      if (lastToggleMs == 0 || (uint32_t)(now - lastToggleMs) >= TAP_DEBOUNCE_MS) {
        g_state.session = !g_state.session;
        lastToggleMs = now ? now : 1;
      }
      newSession = g_state.session;
      portEXIT_CRITICAL(&stateMux);

      Serial.printf("LOG,rfid tap %s session=%d\n", hex, newSession ? 1 : 0);

      mfrc522.PICC_HaltA();        // kartu yang masih menempel tidak terbaca berulang
      mfrc522.PCD_StopCrypto1();
    }
    vTaskDelay(pdMS_TO_TICKS(RFID_POLL_MS));
  }
}

// ==========================================
// 5. TASK LCD (FreeRTOS, Core 1) - hanya status RFID
// ==========================================
static void lcdPrintLine(uint8_t row, const char *text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);   // pad spasi -> tidak ada sisa karakter lama
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void TaskUpdateLCD(void *pvParameters) {
  (void) pvParameters;

  vTaskDelay(pdMS_TO_TICKS(200));   // beri waktu LCD stabil setelah power-up
  lcd.init();
  lcd.backlight();
  lcd.clear();

  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    Snapshot s;
    readState(s);

    char line[32];
    if (!(g_rfidState == RFID_OK && s.session)) lcdPrintLine(0, "RFID STATUS");

    if (g_rfidState == RFID_ERROR) {
      lcdPrintLine(1, "READER ERROR");
    } else if (g_rfidState == RFID_STARTING) {
      lcdPrintLine(1, "STARTING...");
    } else if (s.session) {
      // Sesi aktif: tampilkan BPM + status link, dan data rotasi (potensiometer)
      if (s.bpm > 0) snprintf(line, sizeof(line), "BPM:%-3d ESP:%s", s.bpm, s.link ? "OK" : "LOST");
      else           snprintf(line, sizeof(line), "BPM:--  ESP:%s", s.link ? "OK" : "LOST");
      lcdPrintLine(0, line);

      int pct = map(s.pot, 0, 4095, 0, 100);
      snprintf(line, sizeof(line), "POT:%4d %3d%%", s.pot, pct);
      lcdPrintLine(1, line);
    } else {
      lcdPrintLine(1, "NOT TAPPED");
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LCD_PERIOD_MS));
  }
}

// ==========================================
// 6. SERIAL OUTPUT KE UNITY
// ==========================================
// Builder paket. Mengembalikan panjang string, atau 0 jika buffer tidak cukup.
// Versi 1: COMPACT
static size_t buildCompact(char *buf, size_t n, const Snapshot &s) {
  int len = snprintf(buf, n,
      "D,v=%d,pot=%d,bpm=%d,link=%d,rfid=%s,session=%d,rpm=%d,weight=%d\n",
      PACKET_VERSION, s.pot, s.bpm, s.link ? 1 : 0,
      s.tapped ? s.uid : "0", s.session ? 1 : 0, s.rpm, s.weight);
  return (len > 0 && (size_t)len < n) ? (size_t)len : 0;
}

// Versi 2: JSON. Berakhir '\n' untuk Serial; untuk MQTT potong '\n' terakhir (len-1) sebelum publish.
static size_t buildJson(char *buf, size_t n, const Snapshot &s) {
  int len = snprintf(buf, n,
      "{\"v\":%d,\"pot\":%d,\"bpm\":%d,\"link\":%d,\"rfid\":\"%s\","
      "\"session\":%d,\"rpm\":%d,\"weight\":%d}\n",
      PACKET_VERSION, s.pot, s.bpm, s.link ? 1 : 0,
      s.tapped ? s.uid : "0", s.session ? 1 : 0, s.rpm, s.weight);
  return (len > 0 && (size_t)len < n) ? (size_t)len : 0;
}

static void sendSerialFrame(const Snapshot &s) {
  char buf[192];
#if SERIAL_FORMAT == SERIAL_FORMAT_COMPACT
  size_t len = buildCompact(buf, sizeof(buf), s);
#else
  size_t len = buildJson(buf, sizeof(buf), s);
#endif
  if (len) Serial.write((const uint8_t *)buf, len);   // satu panggilan tulis per paket
}

#if ENABLE_MQTT
#include <PubSubClient.h>   // library "PubSubClient" by Nick O'Leary (dipasang di sini karena ENABLE_MQTT baru didefinisikan di atas)

// ==========================================
// 6b. TASK MQTT (FreeRTOS, Core 0) - publish JSON yang sama dengan versi 2
// ==========================================
static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);

void TaskMQTT(void *pvParameters) {
  (void) pvParameters;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(256);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t lastPublish = 0;
  bool channelLogged = false;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      channelLogged = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    if (!channelLogged) {
      // Channel ini HARUS sama dengan channel Slave (ESP-NOW), lihat catatan di header
      Serial.printf("LOG,wifi ip=%s ch=%d\n", WiFi.localIP().toString().c_str(), WiFi.channel());
      channelLogged = true;
    }

    if (!mqtt.connected()) {
      String id = "d11-master-" + WiFi.macAddress();
      if (mqtt.connect(id.c_str(), NULL, NULL, MQTT_TOPIC_STATUS, 1, true, "offline")) {
        mqtt.publish(MQTT_TOPIC_STATUS, "online", true);
        Serial.println("LOG,mqtt connected");
      } else {
        Serial.printf("LOG,mqtt connect failed rc=%d\n", mqtt.state());
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
    }

    mqtt.loop();

    uint32_t now = millis();
    if (now - lastPublish >= MQTT_PUBLISH_MS) {
      lastPublish = now;
      Snapshot s;
      readState(s);
      char buf[192];
      size_t len = buildJson(buf, sizeof(buf), s);
      if (len) {
        buf[len - 1] = '\0';                 // buang '\n' (khusus Serial)
        mqtt.publish(MQTT_TOPIC_DATA, buf);   // QoS 0
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
#endif

// ==========================================
// 7. SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_POTENTIO, INPUT);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);   // 100 kHz, lebih toleran terhadap kabel jumper

  xTaskCreatePinnedToCore(TaskUpdateLCD, "LCDTask",  4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskRFID,      "RFIDTask", 4096, NULL, 1, NULL, 1);

  WiFi.mode(WIFI_STA);
  Serial.printf("LOG,master mac=%s\n", WiFi.macAddress().c_str());   // salin ke masterAddress di Slave
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("LOG,master ready");
  } else {
    Serial.println("LOG,esp-now init failed");
  }

#if ENABLE_MQTT
  xTaskCreatePinnedToCore(TaskMQTT, "MQTTTask", 6144, NULL, 1, NULL, 0);
#endif
}

void loop() {
  static TickType_t lastWake = xTaskGetTickCount();

  int pot = analogRead(PIN_POTENTIO);

  portENTER_CRITICAL(&stateMux);
  g_state.potRaw = pot;
  portEXIT_CRITICAL(&stateMux);

  Snapshot s;
  readState(s);
  sendSerialFrame(s);

  vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SERIAL_PERIOD_MS));
}

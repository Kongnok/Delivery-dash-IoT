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

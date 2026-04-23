
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DFRobot_SHT20.h>
#include <DFRobot_SHT3x.h> 
#include <esp_task_wdt.h> 
#include <WiFi.h>           
#include <HTTPClient.h>     
#include <time.h>           

// ========================================================
// 1.A. KONFIGURASI WIFI & CLOUD API
// ========================================================
const char* WIFI_SSID = "Cuy";
const char* WIFI_PASS = "ya123456789";

// 1. THINGSPEAK
const char* TS_API_KEY = "VFFKO9JK4NCTRW1U"; 

// 2. GOOGLE SHEETS
const char* GS_WEB_APP_URL = "https://script.google.com/macros/s/AKfycbzrpH2exABCf8OxSgv2MrEA8vh6W6in9rzVYblKaeaca8T8xA7JYLaT1ITwpjFkWu_e/exec";

// 3. SUPABASE (REAL-TIME DATABASE)
const char* SUPABASE_URL = "https://nmyjqvukbypsiksgyxxd.supabase.co";
const char* SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Im5teWpxdnVrYnlwc2lrc2d5eHhkIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzQ3NzA5NzUsImV4cCI6MjA5MDM0Njk3NX0.iyy6i4evTmlLZxZkwPLCyq3f7PlA2w9PDZO4hVNbJGA";

// ========================================================
// 1.B. KONFIGURASI JAM INTERNET (NTP)
// ========================================================
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200; // 25200 detik = GMT+7 (Untuk WIB)
const int   daylightOffset_sec = 0;

namespace HardwarePins {
  constexpr uint8_t I2C_SDA = 4;
  constexpr uint8_t I2C_SCL = 9;
  constexpr uint8_t RELAY_COMPRESSOR = 16;    
  constexpr uint8_t RELAY_VALVE_CS   = 17; 
  constexpr uint8_t RELAY_VALVE_AR   = 18; 
}

constexpr uint8_t LCD_I2C_ADDRESS   = 0x27;
constexpr uint8_t SHT20_I2C_ADDRESS = 0x40;
constexpr uint8_t SHT31_I2C_ADDRESS = 0x45; 

// ========================================================
// 2. PARAMETER WAKTU PROTEKSI & IOT (TIMERS)
// ========================================================
constexpr uint32_t SENSOR_READ_INTERVAL_MS  = 1000;   
constexpr uint32_t COMPRESSOR_MIN_RUN_MS    = 30000;  
constexpr uint32_t ANTI_SHORT_CYCLE_MS      = 180000; 
constexpr uint32_t UNLOADING_START_DELAY_MS = 3000;   
constexpr uint32_t WATCHDOG_TIMEOUT_SEC     = 30; 
constexpr uint32_t TS_SEND_INTERVAL_MS      = 60000; 
constexpr uint32_t TS_RATE_LIMIT_MS         = 16000;
constexpr uint32_t RT_SUPABASE_INTERVAL_MS  = 2000; 
uint32_t lastRealtimeSendTime = 0; 

constexpr float OFFSET_TEMP_CS = 0.0f; 
constexpr float OFFSET_TEMP_AR = 0.0f; 

struct TempRange { float cutIn; float cutOut; };
constexpr TempRange CS_TARGET{29.0f, 28.0f}; 
constexpr TempRange AR_TARGET{30.0f, 29.0f}; 
constexpr float TEMP_HARD_MIN = -20.0f;
constexpr float TEMP_HARD_MAX =  60.0f;

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 20, 4);
DFRobot_SHT20 sht20;
DFRobot_SHT3x sht31(&Wire, SHT31_I2C_ADDRESS); 

enum class RelayStatus : uint8_t { OFF=0, ON=1 };

struct SensorData {
  float tempCS = NAN, humCS = NAN;
  float tempAR = NAN, humAR = NAN;
  bool  isValidCS = false;
  bool  isValidAR = false;
};

struct RelayControl {
  RelayStatus valveCS = RelayStatus::OFF;
  RelayStatus valveAR = RelayStatus::OFF;
  RelayStatus compressor = RelayStatus::OFF;
};

// ========================================================
// 3. FREERTOS DUAL-CORE VARIABLES (DENGAN TIMESTAMP)
// ========================================================
enum class TriggerType { PERIODIC, EVENT, REALTIME };

struct IoTMessage {
  TriggerType trigger;
  SensorData sensor;
  RelayControl relay;
  char timestamp[30]; // Diperbesar sedikit untuk nampung +07:00
};

QueueHandle_t iotQueue; 
TaskHandle_t iotTaskHandle;

RelayControl systemRelays;
RelayControl lastSentRelays; 
uint32_t lastSensorReadTime = 0;
uint32_t compressorLastSwitchTime = 0;
uint32_t lastStrictPeriodicTime = 0; 
uint32_t lastEventTriggerTime = 0;
uint32_t lastWiFiCheckTime = 0; 
uint8_t sht20ErrorCount = 0;
constexpr uint8_t MAX_SENSOR_ERRORS = 3;

// ========================================================
// 5. FUNGSI PERIPHERAL & LOGIKA DASAR
// ========================================================
void setRelayOpenDrain(uint8_t pin, RelayStatus status) {
  if (status == RelayStatus::ON) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); } 
  else { pinMode(pin, INPUT); }
}

const char* getStatusText(RelayStatus status) { return (status == RelayStatus::ON) ? "ON " : "OFF"; }
bool isTemperatureValid(float temp) { return !isnan(temp) && temp >= TEMP_HARD_MIN && temp <= TEMP_HARD_MAX; }
bool isHumidityValid(float hum) { return !isnan(hum) && hum >= 0.0f && hum <= 105.0f; } 

RelayStatus calculateCoolingDemand(RelayStatus currentStatus, float currentTemp, const TempRange& target) {
  if (!isTemperatureValid(currentTemp)) return RelayStatus::OFF; 
  if (currentTemp > target.cutIn)  return RelayStatus::ON;  
  if (currentTemp < target.cutOut) return RelayStatus::OFF; 
  return currentStatus;
}

bool checkI2CConnection(uint8_t address) { Wire.beginTransmission(address); return (Wire.endTransmission() == 0); }
void restartI2CBus() {
  Wire.end(); delay(10);
  Wire.begin(HardwarePins::I2C_SDA, HardwarePins::I2C_SCL); Wire.setClock(100000); delay(10);
}

SensorData updateSensorReadings() {
  SensorData data;
  if (checkI2CConnection(SHT20_I2C_ADDRESS)) {
    float rawTempCS = sht20.readTemperature() + OFFSET_TEMP_CS; 
    float rawHumCS  = sht20.readHumidity();
    data.isValidCS = isTemperatureValid(rawTempCS) && isHumidityValid(rawHumCS);
    if (data.isValidCS) { data.tempCS = rawTempCS; data.humCS = rawHumCS; sht20ErrorCount = 0; } 
    else { sht20ErrorCount++; }
    if (sht20ErrorCount >= MAX_SENSOR_ERRORS) { restartI2CBus(); sht20.initSHT20(); restartI2CBus(); sht20ErrorCount = 0; }
  }

  float rawTempAR = sht31.getTemperatureC(); 
  float rawHumAR  = sht31.getHumidityRH();
  if (rawTempAR > -30.0f && rawTempAR < 100.0f) { 
    rawTempAR += OFFSET_TEMP_AR; 
    data.isValidAR = isTemperatureValid(rawTempAR) && isHumidityValid(rawHumAR);
    if (data.isValidAR) { data.tempAR = rawTempAR; data.humAR = rawHumAR; }
  }
  return data;
}

// ========================================================
// 7, 8, & 9. FUNGSI IOT CLOUD (BERJALAN DI CORE 0)
// ========================================================
void sendToThingSpeak(const SensorData& data, const RelayControl& relays, const char* timestamp) {
  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" + String(TS_API_KEY);
  
  if (String(timestamp) != "NO_TIME") {
    String safeTime = String(timestamp);
    safeTime.replace("+", "%2B"); // Mencegah server bule salah paham
    url += "&created_at=" + safeTime; 
  }
  
  if (data.isValidCS) { url += "&field1=" + String(data.tempCS, 2) + "&field2=" + String(data.humCS, 1); }
  if (data.isValidAR) { url += "&field3=" + String(data.tempAR, 2) + "&field4=" + String(data.humAR, 1); }
  url += "&field5=" + String((uint8_t)relays.valveCS) + "&field6=" + String((uint8_t)relays.valveAR);
  url += "&field7=" + String((uint8_t)relays.compressor) + "&field8=" + String(WiFi.RSSI()); 

  Serial.println("Core 0: Mengirim data ke ThingSpeak...");
  http.begin(url); int httpCode = http.GET(); 
  if (httpCode == 200) Serial.println("Core 0: [OK] ThingSpeak Updated! (" + String(timestamp) + ")");
  else Serial.printf("Core 0: [GAGAL] ThingSpeak Error: %d\n", httpCode);
  http.end();
}

void sendToGoogleSheets(const SensorData& data, const RelayControl& relays, String tabName, const char* timestamp) {
  HTTPClient http;
  String url = String(GS_WEB_APP_URL) + "?tab=" + tabName;
  
  String safeTime = String(timestamp);
  safeTime.replace("+", "%2B"); 
  
  url += "&time=" + safeTime; 
  url += "&tCS="  + (data.isValidCS ? String(data.tempCS, 2) : "ERR");
  url += "&hCS=" + (data.isValidCS ? String(data.humCS, 1) : "ERR");
  url += "&tAR=" + (data.isValidAR ? String(data.tempAR, 2) : "ERR");
  url += "&hAR=" + (data.isValidAR ? String(data.humAR, 1) : "ERR");
  
  if (tabName == "lengkap") {
    url += "&vCS=" + String((uint8_t)relays.valveCS);
    url += "&vAR=" + String((uint8_t)relays.valveAR);
    url += "&comp=" + String((uint8_t)relays.compressor);
  }

  Serial.println("Core 0: Mengirim data ke GS Tab: " + tabName);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(url); int httpCode = http.GET();
  if (httpCode > 0) Serial.println("Core 0: [OK] Google Sheets Updated (" + tabName + ")!");
  else Serial.printf("Core 0: [GAGAL] Sheets Error: %s\n", http.errorToString(httpCode).c_str());
  http.end();
}

String getJSONFloat(float val) { return isnan(val) ? "null" : String(val, 2); }

void sendToSupabase(const SensorData& data, const RelayControl& relays, TriggerType trigger, const char* timestamp) {
  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/cold_storage_logs";
  
  http.begin(url);
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal"); 

  String payload = "{";
  if (String(timestamp) != "NO_TIME") {
    payload += "\"device_time\":\"" + String(timestamp) + "\","; 
  }
  payload += "\"log_type\":\"" + String(trigger == TriggerType::PERIODIC ? "PERIODIC" : "EVENT") + "\",";
  payload += "\"temp_cs\":" + getJSONFloat(data.tempCS) + ",";
  payload += "\"hum_cs\":" + getJSONFloat(data.humCS) + ",";
  payload += "\"temp_ar\":" + getJSONFloat(data.tempAR) + ",";
  payload += "\"hum_ar\":" + getJSONFloat(data.humAR) + ",";
  payload += "\"valve_cs\":" + String((uint8_t)relays.valveCS) + ",";
  payload += "\"valve_ar\":" + String((uint8_t)relays.valveAR) + ",";
  payload += "\"compressor\":" + String((uint8_t)relays.compressor);
  payload += "}";

  Serial.println("Core 0: Mengirim data ke Supabase...");
  int httpCode = http.POST(payload);

  if (httpCode >= 200 && httpCode < 300) {
    Serial.println("Core 0: [OK] Supabase Berhasil Diupdate!");
  } else {
    Serial.printf("Core 0: [GAGAL] Supabase Error: %d\n", httpCode);
    Serial.println("Pesan Supabase: " + http.getString());
  }
  http.end();
}

// FUNGSI  UNTUK REALTIME (UPDATE BARIS ID 1 TERUS MENERUS)
void sendToSupabaseRealtime(const SensorData& data, const RelayControl& relays) {
  HTTPClient http;
  //  URL:  kunci ke id=eq.1 supaya tidak nambah baris baru
  String url = String(SUPABASE_URL) + "/rest/v1/cold_storage_realtime?id=eq.1";
  
  http.begin(url);
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
  http.addHeader("Content-Type", "application/json");

  String payload = "{";
  payload += "\"temp_cs\":" + getJSONFloat(data.tempCS) + ",";
  payload += "\"hum_cs\":" + getJSONFloat(data.humCS) + ",";
  payload += "\"temp_ar\":" + getJSONFloat(data.tempAR) + ",";
  payload += "\"hum_ar\":" + getJSONFloat(data.humAR) + ",";
  payload += "\"valve_cs\":" + String((uint8_t)relays.valveCS) + ",";
  payload += "\"valve_ar\":" + String((uint8_t)relays.valveAR) + ",";
  payload += "\"compressor\":" + String((uint8_t)relays.compressor);
  payload += "}";

  // Pakai PATCH untuk menimpa data yang sudah ada
  http.PATCH(payload); 
  http.end();
}
// ========================================================
// TUGAS INTERNET CLOUD (TASK CORE 0)
// ========================================================
void iotTaskCode(void *pvParameters) {
  #if CONFIG_IDF_TARGET_ESP32
    esp_task_wdt_add(NULL); 
  #endif

  IoTMessage msg;
  uint32_t lastThingSpeakTime = 0; 

  for(;;) {
    // Cek apakah ada pesan masuk dari Core 1
    if (xQueueReceive(iotQueue, &msg, pdMS_TO_TICKS(2000)) == pdPASS) {
      if (WiFi.status() == WL_CONNECTED) {
        
        // ============================================================
        // JALUR 1: DATA REALTIME (Hanya ke Supabase Realtime)
        // ============================================================
        if (msg.trigger == TriggerType::REALTIME) {
          // Panggil fungsi PATCH yang baru dibuat
          sendToSupabaseRealtime(msg.sensor, msg.relay);
          //  tidak kirim ke TS atau GS di sini agar tidak kena rate limit
        } 
        
        // ============================================================
        // JALUR 2: DATA LOG / EVENT (Ke Semua Cloud - Logika Asli)
        // ============================================================
        else {
          Serial.println("\nCore 0: Memulai pengiriman LOG/EVENT waktu: " + String(msg.timestamp));

          uint32_t currentCore0Time = millis();
          
          // 1. Kirim ke ThingSpeak ( pakai limit 16 detik)
          if (currentCore0Time - lastThingSpeakTime >= TS_RATE_LIMIT_MS || lastThingSpeakTime == 0) {
            sendToThingSpeak(msg.sensor, msg.relay, msg.timestamp);
            lastThingSpeakTime = currentCore0Time;
          } else {
            Serial.println("Core 0: ThingSpeak dilewati (Rate Limit aktif).");
          }
          
          // 2. Kirim ke Google Sheets
          sendToGoogleSheets(msg.sensor, msg.relay, "lengkap", msg.timestamp); 
          if (msg.trigger == TriggerType::PERIODIC) {
            sendToGoogleSheets(msg.sensor, msg.relay, "suhu", msg.timestamp);
          }

          // 3. Kirim ke Supabase Logs (Insert baris baru untuk History)
          sendToSupabase(msg.sensor, msg.relay, msg.trigger, msg.timestamp);
        }
      }
    }
    
    #if CONFIG_IDF_TARGET_ESP32
      esp_task_wdt_reset(); 
    #endif
  }
}

// ========================================================
// FUNGSI TAMPILAN LCD & JAM
// ========================================================
void printFormattedFloat(float value, uint8_t width, uint8_t prec) {
  if (isnan(value)) { lcd.print("ERR"); return; }
  char buf[16]; dtostrf(value, width, prec, buf); lcd.print(buf);
}

void refreshLCDDisplay(const SensorData& data, const RelayControl& relays) {
  uint32_t currentTime = millis();

  lcd.setCursor(0, 0); lcd.print("CS T:"); lcd.setCursor(5, 0); printFormattedFloat(data.isValidCS ? data.tempCS : NAN, 5, 1);
  lcd.setCursor(11, 0); lcd.print(" H:"); lcd.setCursor(14, 0); printFormattedFloat(data.isValidCS ? data.humCS : NAN, 5, 0);
  lcd.setCursor(0, 1); lcd.print("AR T:"); lcd.setCursor(5, 1); printFormattedFloat(data.isValidAR ? data.tempAR : NAN, 5, 1);
  lcd.setCursor(11, 1); lcd.print(" H:"); lcd.setCursor(14, 1); printFormattedFloat(data.isValidAR ? data.humAR : NAN, 5, 0);
  lcd.setCursor(0, 2); lcd.print("CS:"); lcd.print(getStatusText(relays.valveCS));
  lcd.print(" AR:"); lcd.print(getStatusText(relays.valveAR)); lcd.print(" CP:"); lcd.print(getStatusText(relays.compressor));

  lcd.setCursor(0, 3); char statusMessage[21]; 
  if (relays.compressor == RelayStatus::ON) {
    uint32_t runTimeSec = (currentTime - compressorLastSwitchTime) / 1000;
    snprintf(statusMessage, sizeof(statusMessage), "Run Time : %02u:%02u    ", runTimeSec / 60, runTimeSec % 60);
  } else {
    if (currentTime - compressorLastSwitchTime < ANTI_SHORT_CYCLE_MS) {
      uint32_t remainSec = (ANTI_SHORT_CYCLE_MS - (currentTime - compressorLastSwitchTime)) / 1000;
      snprintf(statusMessage, sizeof(statusMessage), "Wait(ASC): %02u:%02u    ", remainSec / 60, remainSec % 60); 
    } else {
      if (relays.valveCS == RelayStatus::ON || relays.valveAR == RelayStatus::ON) snprintf(statusMessage, sizeof(statusMessage), "CP Starting...      ");
      else snprintf(statusMessage, sizeof(statusMessage), "System IDLE / READY ");
    }
  }
  lcd.print(statusMessage);
}

void getCurrentTimeString(char* buffer, size_t maxLen) {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 10)) {
    // Format Waktu Universal (ISO 8601) 
    strftime(buffer, maxLen, "%Y-%m-%dT%H:%M:%S+07:00", &timeinfo);
  } else {
    snprintf(buffer, maxLen, "NO_TIME");
  }
}

// ========================================================
// SETUP UTAMA
// ========================================================
void setup() {
  Serial.begin(115200); delay(200);

  setRelayOpenDrain(HardwarePins::RELAY_COMPRESSOR, RelayStatus::OFF);
  setRelayOpenDrain(HardwarePins::RELAY_VALVE_CS, RelayStatus::OFF);
  setRelayOpenDrain(HardwarePins::RELAY_VALVE_AR, RelayStatus::OFF);

  #if CONFIG_IDF_TARGET_ESP32
    esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true); esp_task_wdt_add(NULL); 
  #endif

  iotQueue = xQueueCreate(10, sizeof(IoTMessage)); 
  
  xTaskCreatePinnedToCore(
    iotTaskCode,   
    "IoT_Task",    
    10000,         
    NULL,          
    1,             
    &iotTaskHandle,
    0              
  );

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Menghubungkan ke WiFi");
  uint32_t startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
    delay(500); Serial.print(".");
    #if CONFIG_IDF_TARGET_ESP32
      esp_task_wdt_reset(); 
    #endif
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Terhubung!"); configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); Serial.println("Menyinkronkan waktu NTP...");
  } else { Serial.println("\nWiFi Gagal, lanjut offline."); }

  Wire.begin(HardwarePins::I2C_SDA, HardwarePins::I2C_SCL); Wire.setClock(100000); delay(50);
  sht20.initSHT20(); delay(50); restartI2CBus(); sht31.begin(); 
  
  lcd.init(); lcd.backlight(); lcd.setCursor(0,0); lcd.print("SYSTEM STARTING..."); delay(1000); lcd.clear();

  lastSensorReadTime = millis();
  compressorLastSwitchTime = millis(); 
  lastStrictPeriodicTime = millis(); 
  lastWiFiCheckTime = millis();
}

// ========================================================
// LOOP UTAMA (CORE 1: MANDOR MESIN & LCD)
// ========================================================
void loop() {
  #if CONFIG_IDF_TARGET_ESP32
    esp_task_wdt_reset(); 
  #endif

  uint32_t currentTime = millis();

  // --- PEMANTAU WIFI & SINKRONISASI JAM OTOMATIS ---
  static bool wasWiFiConnected = false;
  bool isWiFiConnected = (WiFi.status() == WL_CONNECTED);

  if (!isWiFiConnected && (currentTime - lastWiFiCheckTime >= 20000)) {
    Serial.println("Sistem: WiFi terputus! Mencoba *reconnect* di background...");
    WiFi.disconnect(); WiFi.reconnect(); lastWiFiCheckTime = currentTime;
  }

  // FIX bug "NO_TIME" saat reconnect
  if (isWiFiConnected && !wasWiFiConnected) {
    Serial.println("Sistem: WiFi Terhubung! Memaksa sinkronisasi ulang jam NTP...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }
  wasWiFiConnected = isWiFiConnected;

  if (currentTime - lastSensorReadTime < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadTime = currentTime;

  SensorData currentSensorData = updateSensorReadings();
  static uint32_t valveOpenTime = 0; 
  
  RelayStatus demandCS = currentSensorData.isValidCS ? calculateCoolingDemand(systemRelays.valveCS, currentSensorData.tempCS, CS_TARGET) : RelayStatus::OFF;
  RelayStatus demandAR = currentSensorData.isValidAR ? calculateCoolingDemand(systemRelays.valveAR, currentSensorData.tempAR, AR_TARGET) : RelayStatus::OFF;
  bool isCoolingNeeded = (demandCS == RelayStatus::ON || demandAR == RelayStatus::ON);
  bool isCompressorInLockout = (systemRelays.compressor == RelayStatus::OFF) && (currentTime - compressorLastSwitchTime < ANTI_SHORT_CYCLE_MS);

  if (isCoolingNeeded) {
    if (isCompressorInLockout) {
      systemRelays.valveCS = RelayStatus::OFF; systemRelays.valveAR = RelayStatus::OFF;
    } else {
      if (systemRelays.valveCS == RelayStatus::OFF && systemRelays.valveAR == RelayStatus::OFF) { valveOpenTime = currentTime; }
      systemRelays.valveCS = demandCS; systemRelays.valveAR = demandAR;
      if (currentTime - valveOpenTime >= UNLOADING_START_DELAY_MS) {
        if (systemRelays.compressor == RelayStatus::OFF) { systemRelays.compressor = RelayStatus::ON; compressorLastSwitchTime = currentTime; }
      }
    }
  } else {
    systemRelays.valveCS = RelayStatus::OFF; systemRelays.valveAR = RelayStatus::OFF;
    if (systemRelays.compressor == RelayStatus::ON) {
      if (currentTime - compressorLastSwitchTime >= COMPRESSOR_MIN_RUN_MS) { systemRelays.compressor = RelayStatus::OFF; compressorLastSwitchTime = currentTime; }
    } else { systemRelays.compressor = RelayStatus::OFF; }
  }

  setRelayOpenDrain(HardwarePins::RELAY_VALVE_CS, systemRelays.valveCS);
  setRelayOpenDrain(HardwarePins::RELAY_VALVE_AR, systemRelays.valveAR);
  setRelayOpenDrain(HardwarePins::RELAY_COMPRESSOR, systemRelays.compressor);

  refreshLCDDisplay(currentSensorData, systemRelays);

  char timeStringBuff[30]; 
  getCurrentTimeString(timeStringBuff, sizeof(timeStringBuff)); 

  Serial.printf("[%s] CS: %.2f C, %.1f%% | AR: %.2f C, %.1f%% | V_CS:%d V_AR:%d CP:%d | WiFi: %d dBm\n",
                timeStringBuff, 
                currentSensorData.isValidCS ? currentSensorData.tempCS : 0.0, currentSensorData.isValidCS ? currentSensorData.humCS : 0.0,
                currentSensorData.isValidAR ? currentSensorData.tempAR : 0.0, currentSensorData.isValidAR ? currentSensorData.humAR : 0.0,
                (uint8_t)systemRelays.valveCS, (uint8_t)systemRelays.valveAR, (uint8_t)systemRelays.compressor, WiFi.RSSI());
  
  // --- LOGIKA UNTUK REALTIME (SETIAP 2 DETIK) ---
  if (currentTime - lastRealtimeSendTime >= RT_SUPABASE_INTERVAL_MS) {
    IoTMessage rtMsg;
    rtMsg.trigger = TriggerType::REALTIME; 
    rtMsg.sensor = currentSensorData;
    rtMsg.relay = systemRelays;
    // Realtime tidak butuh timestamp karena hanya update angka di tabel tunggal
    xQueueSend(iotQueue, &rtMsg, 0); 
    lastRealtimeSendTime = currentTime;
  }

  static bool pendingEventSend = false;

  bool isRelayChanged = (systemRelays.compressor != lastSentRelays.compressor) ||
                        (systemRelays.valveCS != lastSentRelays.valveCS) ||
                        (systemRelays.valveAR != lastSentRelays.valveAR);

  if (isRelayChanged && !pendingEventSend) {
    pendingEventSend = true; lastEventTriggerTime = currentTime; lastSentRelays = systemRelays; 
  }

  if (currentTime - lastStrictPeriodicTime >= TS_SEND_INTERVAL_MS) {
    IoTMessage newMsg;
    newMsg.trigger = TriggerType::PERIODIC;
    newMsg.sensor = currentSensorData;
    newMsg.relay = systemRelays;
    strncpy(newMsg.timestamp, timeStringBuff, sizeof(newMsg.timestamp)); 
    
    xQueueSend(iotQueue, &newMsg, 0); 
    lastStrictPeriodicTime = currentTime;
    pendingEventSend = false; 
  }
  else if (pendingEventSend && (currentTime - lastEventTriggerTime >= 2500)) {
    IoTMessage newMsg;
    newMsg.trigger = TriggerType::EVENT;
    newMsg.sensor = currentSensorData;
    newMsg.relay = systemRelays;
    strncpy(newMsg.timestamp, timeStringBuff, sizeof(newMsg.timestamp)); 
    
    xQueueSend(iotQueue, &newMsg, 0); 
    pendingEventSend = false;
  }
}
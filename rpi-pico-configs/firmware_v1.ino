// ================================================================
// FULL AQ MONITOR — GAS + PM (PMS + OPC-N3) + CO2 + SOUND + MQTT
// ================================================================
// VERSION: v4 — OPC debug tracing
// ================================================================

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <stdlib.h>
#include <math.h>

// ================================================================
// WIFI + MQTT CREDENTIALS
// ================================================================
namespace Net {
    const char *WIFI_SSID     = "iPhone 14(Mansur)";
    const char *WIFI_PASS     = "x9vb20x9";

    const char *MQTT_HOST     = "969c1686c9e94227adf987ab9bb31fb9.s1.eu.hivemq.cloud";
    const int   MQTT_PORT     = 8883;
    const char *MQTT_USER     = "joonas";
    const char *MQTT_PASS     = "Joonas10";
    const char *MQTT_TOPIC    = "aq/data";
    const char *MQTT_STATUS   = "aq/status";

    constexpr unsigned long WIFI_RETRY_MS   = 30000UL;
    constexpr unsigned long MQTT_RETRY_MS   = 10000UL;
    constexpr unsigned long MQTT_PUBLISH_MS = 5000UL;
}

constexpr bool SWAP_NO2 = true;

// ================================================================
// CONFIGURATION
// ================================================================
namespace Config {
    constexpr uint8_t SDA_PIN = 8;
    constexpr uint8_t SCL_PIN = 9;
    constexpr float   MV_PER_LSB = 0.03125f;

    constexpr uint8_t DHT_PIN = 12;

    constexpr uint8_t CO2_TX = 0;
    constexpr uint8_t CO2_RX = 1;

    constexpr uint8_t PMS_TX = 4;
    constexpr uint8_t PMS_RX = 5;

    constexpr uint8_t MIC_PIN = 26;

    constexpr uint8_t  OPC_CS_PIN   = 17;
    constexpr uint8_t  OPC_SCK_PIN  = 18;
    constexpr uint8_t  OPC_MOSI_PIN = 19;
    constexpr uint8_t  OPC_MISO_PIN = 16;
    constexpr uint32_t OPC_SPI_SPEED = 300000;

    constexpr float SENS_NO2   = 309.0f;
    constexpr float SENS_OX    = 298.0f;
    constexpr float RGAIN      = 47.0f;
    constexpr float K_OX_NO2   = 0.7f;
    constexpr float S_NO2_MV   = SENS_NO2 * RGAIN / 1000.0f;
    constexpr float S_OX_MV    = SENS_OX  * RGAIN / 1000.0f;

    constexpr uint8_t CAL_SAMPLES = 120;
    constexpr float   BL_ALPHA    = 0.002f;
    constexpr float   BL_LOCK_PPB = 15.0f;
    constexpr float   ELEC_MIN_MV = 150.0f;
    constexpr float   ELEC_MAX_MV = 800.0f;

    constexpr unsigned long SAMPLE_MS    = 1000UL;
    constexpr unsigned long CO2_MS       = 5000UL;
    constexpr unsigned long DHT_MS       = 2500UL;
    constexpr unsigned long MIC_MS       = 1000UL;
    constexpr unsigned long OPC_MS       = 5000UL;
    constexpr unsigned long CO2_STALE_MS = 20000UL;
    constexpr unsigned long PM_STALE_MS  = 10000UL;
    constexpr unsigned long OPC_STALE_MS = 30000UL;  // increased from 15s to 30s

    constexpr unsigned long CO2_RESP_MS  = 600UL;

    constexpr float ADC_MV_PER_COUNT = 3300.0f / 4096.0f;
    constexpr int   MIC_SAMPLES      = 256;
    constexpr float MIC_ALPHA_RISE   = 0.8f;
    constexpr float MIC_ALPHA_FALL   = 0.05f;
}

// ================================================================
// ADS1115 CHANNEL MAP
// ================================================================
namespace CH {
    constexpr uint8_t WE_OX   = 0;
    constexpr uint8_t AUX_OX  = 1;
    constexpr uint8_t WE_NO2  = SWAP_NO2 ? 3 : 2;
    constexpr uint8_t AUX_NO2 = SWAP_NO2 ? 2 : 3;
}

Adafruit_ADS1115 ads;
bool ads_ok = false;

// ================================================================
// WIFI + MQTT OBJECTS
// ================================================================
WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastMqttPublish = 0;
bool wifi_connected = false;
bool mqtt_connected = false;

// ================================================================
// TEMPERATURE CORRECTION — AAN803 full piecewise table
// ================================================================
static inline float nNO2(float t) {
    if (t < 0.0f)  return 1.18f;
    if (t < 10.0f) return 1.00f;
    if (t < 20.0f) return 0.76f;
    if (t < 30.0f) return 0.68f;
    if (t < 40.0f) return 0.23f;
    return 0.00f;
}
static inline float nOX(float t) {
    if (t < 0.0f)  return 0.18f;
    if (t < 10.0f) return 0.77f;
    if (t < 20.0f) return 1.56f;
    if (t < 30.0f) return 1.56f;
    if (t < 40.0f) return 1.56f;
    return 2.85f;
}

// ================================================================
// SOFTWARE CLOCK
// ================================================================
static uint32_t clock_base_unix = 0;
static uint32_t clock_base_ms   = 0;

static bool swcIsLeapYear(int y) {
    return (y % 400 == 0) || (y % 4 == 0 && y % 100 != 0);
}
static int swcMonthNum(const char *m) {
    switch (m[0]) {
        case 'J': if (m[1]=='a') return 1; return (m[2]=='n') ? 6 : 7;
        case 'F': return 2;
        case 'M': return (m[2]=='r') ? 3 : 5;
        case 'A': return (m[1]=='p') ? 4 : 8;
        case 'S': return 9;  case 'O': return 10;
        case 'N': return 11; default:  return 12;
    }
}
static uint32_t swcToUnix(int yr, int mo, int dy, int h, int mi, int s) {
    static const uint16_t dbm[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
    uint32_t days = 0;
    for (int y = 1970; y < yr; y++) days += swcIsLeapYear(y) ? 366 : 365;
    days += dbm[mo - 1];
    if (mo > 2 && swcIsLeapYear(yr)) days++;
    days += (uint32_t)(dy - 1);
    return days * 86400UL + (uint32_t)h * 3600UL + (uint32_t)mi * 60UL + s;
}
static uint32_t swcCompile() {
    const char *d = __DATE__, *t = __TIME__;
    char mon[4] = {d[0], d[1], d[2], '\0'};
    return swcToUnix(atoi(d+7), swcMonthNum(mon), atoi(d+4),
        (t[0]-'0')*10+(t[1]-'0'),
        (t[3]-'0')*10+(t[4]-'0'),
        (t[6]-'0')*10+(t[7]-'0'));
}
static void     swcInit()          { clock_base_unix = swcCompile(); clock_base_ms = millis(); }
static void     swcSet(uint32_t t) { clock_base_unix = t; clock_base_ms = millis(); }
static uint32_t swcNow()           { return clock_base_unix + (millis() - clock_base_ms) / 1000UL; }

static void handleSerialTimeSync() {
    if (!Serial) return;
    static char buf[16]; static uint8_t idx = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx > 1 && buf[0] == 'T') {
                buf[idx] = '\0';
                uint32_t ts = strtoul(buf + 1, nullptr, 10);
                if (ts > 1600000000UL) swcSet(ts);
            }
            idx = 0;
        } else if (idx < sizeof(buf) - 1) { buf[idx++] = c; }
        else { idx = 0; }
    }
}

// ================================================================
// CALIBRATION STATE
// ================================================================
struct CalState {
    float   WE_NO2 = 0, AUX_NO2 = 0, WE_OX = 0, AUX_OX = 0;
    bool    valid  = false;
    uint8_t count  = 0;
    float   sum_WE_NO2 = 0, sum_AUX_NO2 = 0, sum_WE_OX = 0, sum_AUX_OX = 0;
} cal;

// ================================================================
// GLOBAL STATE
// ================================================================
float temp_c = 20.0f, rh_pct = 50.0f;

int   co2_ppm = -1;   bool co2_valid = false;   unsigned long lastCO2Frame = 0;

int   pm1_0 = -1, pm2_5 = -1, pm10 = -1;
bool  pm_valid = false; unsigned long lastPMFrame = 0;

// OPC-N3 state
bool     opc_initialized = false;
bool     opc_data_valid  = false;
float    opc_pm1   = 0.0f;
float    opc_pm2_5 = 0.0f;
float    opc_pm10  = 0.0f;
unsigned long opc_lastRead  = 0;
unsigned long opc_lastFrame = 0;
uint32_t opc_fail_handshake = 0;
uint32_t opc_fail_sanity    = 0;
uint32_t opc_success        = 0;
uint32_t opc_not_ready      = 0;
uint32_t opc_stale_count    = 0;

float mic_mv = 0.0f;
float baseline_no2 = 0.0f, baseline_ox = 0.0f;
unsigned long lastSample = 0, lastCO2 = 0, lastDHT = 0, lastMic = 0;
static const uint8_t CO2_CMD[4] = {0x11, 0x01, 0x01, 0xED};

// Last computed values for JSON output
float last_no2_out = 0, last_o3_out = 0, last_ox_out = 0;
float last_sig_no2 = 0, last_sig_ox = 0;
float last_WE_no2 = 0, last_AUX_no2 = 0, last_WE_ox = 0, last_AUX_ox = 0;
bool  last_hw_warn = false;

// ================================================================
// JSON BUFFER
// ================================================================
static char jsonBuf[768];

// ================================================================
// ADS1115
// ================================================================
static inline float readADS(uint8_t ch) {
    if (!ads_ok) return -1.0f;
    int16_t raw = ads.readADC_SingleEnded(ch);
    if (raw < -32000 || raw > 32000) return -1.0f;
    return raw * Config::MV_PER_LSB;
}

// ================================================================
// DHT11
// ================================================================
static bool waitPin(uint8_t pin, int state, uint32_t us) {
    uint32_t t = micros();
    while (digitalRead(pin) != state) if ((micros()-t) > us) return false;
    return true;
}
static uint32_t measurePin(uint8_t pin, int state, uint32_t us) {
    uint32_t t = micros();
    while (digitalRead(pin) == state) if ((micros()-t) > us) return 0;
    return micros() - t;
}
static bool readDHT11(float &tc, float &rh) {
    uint8_t data[5] = {}, pin = Config::DHT_PIN;
    pinMode(pin, INPUT_PULLUP); delay(2);
    if (digitalRead(pin) == LOW) return false;
    pinMode(pin, OUTPUT); digitalWrite(pin, LOW); delay(25);
    digitalWrite(pin, HIGH); delayMicroseconds(30);
    pinMode(pin, INPUT_PULLUP);
    if (!waitPin(pin, LOW,  200)) return false;
    if ( measurePin(pin, LOW,  200) == 0) return false;
    if (!waitPin(pin, HIGH, 200)) return false;
    if ( measurePin(pin, HIGH, 200) == 0) return false;
    for (uint8_t i = 0; i < 40; i++) {
        if (measurePin(pin, LOW, 200) == 0) return false;
        uint32_t hi = measurePin(pin, HIGH, 200);
        if (!hi) return false;
        data[i >> 3] <<= 1;
        if (hi > 45) data[i >> 3] |= 1;
    }
    if ((uint8_t)(data[0]+data[1]+data[2]+data[3]) != data[4]) return false;
    rh = (float)data[0];
    tc = (float)(data[2] & 0x7F);
    if (data[3] & 0x80) tc = -tc;
    return true;
}
static void readDHT() {
    if (millis() - lastDHT < Config::DHT_MS) return;
    lastDHT = millis();
    float t, h;
    if (readDHT11(t, h)) {
        temp_c = 0.9f*temp_c + 0.1f*t;
        rh_pct = 0.9f*rh_pct + 0.1f*h;
    }
}

// ================================================================
// ADMP401
// ================================================================
static void readMic() {
    if (millis() - lastMic < Config::MIC_MS) return;
    lastMic = millis();

    static uint16_t samples[Config::MIC_SAMPLES];
    int32_t sum = 0;
    for (int i = 0; i < Config::MIC_SAMPLES; i++) {
        samples[i] = (uint16_t)analogRead(Config::MIC_PIN);
        sum += samples[i];
    }
    int32_t mean = sum / Config::MIC_SAMPLES;

    int64_t sq = 0;
    for (int i = 0; i < Config::MIC_SAMPLES; i++) {
        int32_t s = (int32_t)samples[i] - mean;
        sq += (int64_t)s * s;
    }
    float rms_counts = sqrtf((float)sq / Config::MIC_SAMPLES);
    float new_mv     = rms_counts * Config::ADC_MV_PER_COUNT;

    if (mic_mv == 0.0f) {
        mic_mv = new_mv;
    } else {
        float alpha = (new_mv > mic_mv) ? Config::MIC_ALPHA_RISE : Config::MIC_ALPHA_FALL;
        mic_mv = (1.0f - alpha) * mic_mv + alpha * new_mv;
    }
}

// ================================================================
// CM1109 CO2
// ================================================================
static bool pollCO2(unsigned long timeout_ms) {
    while (Serial1.available()) Serial1.read();
    Serial1.write(CO2_CMD, 4);
    Serial1.flush();
    uint8_t resp[8] = {}; uint8_t idx = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < timeout_ms && idx < 8)
        if (Serial1.available()) resp[idx++] = Serial1.read();

    if (idx < 8) return false;
    if (resp[0] != 0x16 || resp[1] != 0x05 || resp[2] != 0x01) return false;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) sum += resp[i];
    if ((sum & 0xFF) != 0) return false;
    co2_ppm = (resp[3] << 8) | resp[4];
    co2_valid = true; lastCO2Frame = millis();
    return true;
}

static void readCO2() {
    if (millis() - lastCO2 < Config::CO2_MS) return;
    lastCO2 = millis();
    pollCO2(Config::CO2_RESP_MS);
}

// ================================================================
// PMS (Plantower)
// ================================================================
static void readPMS() {
    static uint8_t frame[32], idx = 0;
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        if      (idx == 0) { if (b != 0x42) continue; }
        else if (idx == 1) { if (b != 0x4D) { idx = 0; continue; } }
        frame[idx++] = b;
        if (idx == 32) {
            idx = 0;
            uint16_t chk = 0;
            for (uint8_t i = 0; i < 30; i++) chk += frame[i];
            if (chk == ((uint16_t)(frame[30] << 8) | frame[31])) {
                pm1_0 = (frame[10] << 8) | frame[11];
                pm2_5 = (frame[12] << 8) | frame[13];
                pm10  = (frame[14] << 8) | frame[15];
                pm_valid = true; lastPMFrame = millis();
            }
        }
    }
}

// ================================================================
// OPC-N3 (Alphasense) — SPI driver
// ================================================================
namespace OPC_CMD {
    constexpr uint8_t POWER_CTRL     = 0x03;
    constexpr uint8_t FAN_ON         = 0x03;
    constexpr uint8_t FAN_OFF        = 0x02;
    constexpr uint8_t LASER_POT_ON   = 0x05;
    constexpr uint8_t LASER_POT_OFF  = 0x04;
    constexpr uint8_t LASER_SW_ON    = 0x07;
    constexpr uint8_t LASER_SW_OFF   = 0x06;
    constexpr uint8_t HIGH_GAIN      = 0x10;
    constexpr uint8_t LOW_GAIN       = 0x11;
    constexpr uint8_t READ_HISTOGRAM = 0x30;
    constexpr uint8_t READ_PM        = 0x32;
    constexpr uint8_t READ_DAC_POWER = 0x13;
    constexpr uint8_t VALID_1        = 0x31;
    constexpr uint8_t VALID_2        = 0xF3;
}

static void opcBeginTransfer() {
    digitalWrite(Config::OPC_CS_PIN, LOW);
    delayMicroseconds(1);
}

static void opcEndTransfer() {
    delayMicroseconds(1);
    digitalWrite(Config::OPC_CS_PIN, HIGH);
}

static bool opcSendCommand(uint8_t cmdByte, uint8_t subByte) {
    uint8_t r1, r2;
    opcBeginTransfer();
    r1 = SPI.transfer(cmdByte);
    delay(10);
    r2 = SPI.transfer(subByte);
    opcEndTransfer();
    return (r1 == OPC_CMD::VALID_1 && r2 == OPC_CMD::VALID_2);
}

static bool opcReadPM(float &p1, float &p25, float &p10) {
    uint8_t r1, r2;
    uint8_t data[14];

    opcBeginTransfer();
    r1 = SPI.transfer(OPC_CMD::READ_PM);
    delay(10);
    r2 = SPI.transfer(OPC_CMD::READ_PM);

    if (r1 != OPC_CMD::VALID_1 || r2 != OPC_CMD::VALID_2) {
        for (int i = 0; i < 14; i++) {
            delayMicroseconds(10);
            SPI.transfer(OPC_CMD::READ_PM);
        }
        opcEndTransfer();
        opc_fail_handshake++;
        return false;
    }

    for (int i = 0; i < 14; i++) {
        delayMicroseconds(10);
        data[i] = SPI.transfer(OPC_CMD::READ_PM);
    }
    opcEndTransfer();

    // "Not ready" — 12 zero bytes + nonzero checksum
    bool all_zero = true;
    for (int i = 0; i < 12; i++) {
        if (data[i] != 0x00) { all_zero = false; break; }
    }
    if (all_zero) {
        opc_not_ready++;
        return false;
    }

    // Parse floats
    memcpy(&p1,  &data[0], 4);
    memcpy(&p25, &data[4], 4);
    memcpy(&p10, &data[8], 4);

    // Sanity
    if (isnan(p1) || isnan(p25) || isnan(p10)) { opc_fail_sanity++; return false; }
    if (p1 < 0.0f || p25 < 0.0f || p10 < 0.0f) { opc_fail_sanity++; return false; }
    if (p10 > 10000.0f) { opc_fail_sanity++; return false; }

    // Relaxed plausibility — only reject if grossly wrong
    if (p25 > 0.001f && p1 > p25 * 3.0f) { opc_fail_sanity++; return false; }
    if (p10 > 0.001f && p25 > p10 * 3.0f) { opc_fail_sanity++; return false; }

    opc_success++;
    return true;
}

static bool opcReadHistogram() {
    uint8_t r1, r2;
    opcBeginTransfer();
    r1 = SPI.transfer(OPC_CMD::READ_HISTOGRAM);
    delay(10);
    r2 = SPI.transfer(OPC_CMD::READ_HISTOGRAM);
    for (int i = 0; i < 86; i++) {
        delayMicroseconds(10);
        SPI.transfer(OPC_CMD::READ_HISTOGRAM);
    }
    opcEndTransfer();
    return (r1 == OPC_CMD::VALID_1 && r2 == OPC_CMD::VALID_2);
}

static void opcSetup() {
    if (Serial) Serial.println(F("{\"event\":\"opc_setup\",\"phase\":\"begin\"}"));

    pinMode(Config::OPC_CS_PIN, OUTPUT);
    digitalWrite(Config::OPC_CS_PIN, HIGH);

    SPI.setSCK(Config::OPC_SCK_PIN);
    SPI.setTX(Config::OPC_MOSI_PIN);
    SPI.setRX(Config::OPC_MISO_PIN);
    SPI.begin();
    SPI.beginTransaction(SPISettings(Config::OPC_SPI_SPEED, MSBFIRST, SPI_MODE1));

    delay(3000);
    opcSendCommand(OPC_CMD::POWER_CTRL, OPC_CMD::FAN_ON);
    delay(1000);
    opcSendCommand(OPC_CMD::POWER_CTRL, OPC_CMD::LASER_POT_ON);
    delay(1000);
    opcSendCommand(OPC_CMD::POWER_CTRL, OPC_CMD::LASER_SW_ON);
    delay(1000);
    opcSendCommand(OPC_CMD::POWER_CTRL, OPC_CMD::HIGH_GAIN);
    delay(1000);
    opcReadHistogram();
    delay(2000);

    opc_initialized = true;
    opc_lastRead = millis();

    if (Serial) Serial.println(F("{\"event\":\"opc_setup\",\"phase\":\"complete\"}"));
}

static void readOPC() {
    if (!opc_initialized) return;
    if (millis() - opc_lastRead < Config::OPC_MS) return;
    opc_lastRead = millis();

    float p1, p25, p10;
    if (opcReadPM(p1, p25, p10)) {
        opc_pm1   = p1;
        opc_pm2_5 = p25;
        opc_pm10  = p10;
        opc_data_valid = true;
        opc_lastFrame  = millis();

        // DEBUG: confirm values stored
        if (Serial) {
            Serial.print(F("{\"event\":\"opc_read_ok\",\"pm1\":"));
            Serial.print(opc_pm1, 4);
            Serial.print(F(",\"pm2_5\":"));
            Serial.print(opc_pm2_5, 4);
            Serial.print(F(",\"pm10\":"));
            Serial.print(opc_pm10, 4);
            Serial.print(F(",\"valid\":true"));
            Serial.print(F(",\"frame_ms\":"));
            Serial.print(opc_lastFrame);
            Serial.print(F(",\"total_ok\":"));
            Serial.print(opc_success);
            Serial.println(F("}"));
        }
    }
}

// ================================================================
// CALIBRATION
// ================================================================
static bool updateCal(float we_no2, float aux_no2, float we_ox, float aux_ox) {
    if (we_no2  < Config::ELEC_MIN_MV || we_no2  > Config::ELEC_MAX_MV ||
        aux_no2 < Config::ELEC_MIN_MV || aux_no2 > Config::ELEC_MAX_MV ||
        we_ox   < Config::ELEC_MIN_MV || we_ox   > Config::ELEC_MAX_MV ||
        aux_ox  < Config::ELEC_MIN_MV || aux_ox  > Config::ELEC_MAX_MV) return false;
    cal.sum_WE_NO2 += we_no2; cal.sum_AUX_NO2 += aux_no2;
    cal.sum_WE_OX  += we_ox;  cal.sum_AUX_OX  += aux_ox;
    cal.count++;
    if (cal.count >= Config::CAL_SAMPLES) {
        cal.WE_NO2  = cal.sum_WE_NO2  / cal.count;
        cal.AUX_NO2 = cal.sum_AUX_NO2 / cal.count;
        cal.WE_OX   = cal.sum_WE_OX   / cal.count;
        cal.AUX_OX  = cal.sum_AUX_OX  / cal.count;
        cal.valid   = true;
        cal.sum_WE_NO2 = cal.sum_AUX_NO2 = cal.sum_WE_OX = cal.sum_AUX_OX = 0;
        cal.count = 0;
        if (Serial) {
            Serial.print(F("{\"event\":\"cal_done\""));
            Serial.print(F(",\"zero_WE_no2_mV\":"));  Serial.print(cal.WE_NO2,  3);
            Serial.print(F(",\"zero_AUX_no2_mV\":")); Serial.print(cal.AUX_NO2, 3);
            Serial.print(F(",\"zero_WE_ox_mV\":"));   Serial.print(cal.WE_OX,   3);
            Serial.print(F(",\"zero_AUX_ox_mV\":"));  Serial.print(cal.AUX_OX,  3);
            Serial.println(F("}"));
        }
        return true;
    }
    return false;
}

// ================================================================
// WIFI
// ================================================================
static void wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) { wifi_connected = true; return; }
    if (millis() - lastWifiAttempt < Net::WIFI_RETRY_MS && lastWifiAttempt != 0) return;
    lastWifiAttempt = millis();
    wifi_connected = false;

    WiFi.mode(WIFI_STA);
    WiFi.begin(Net::WIFI_SSID, Net::WIFI_PASS);

    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
        delay(250);
    }

    wifi_connected = (WiFi.status() == WL_CONNECTED);
    if (Serial) {
        Serial.print(F("{\"event\":\"wifi\",\"status\":\""));
        Serial.print(wifi_connected ? F("connected") : F("failed"));
        Serial.println(F("\"}"));
    }
}

// ================================================================
// MQTT
// ================================================================
static void mqttConnect() {
    if (!wifi_connected) return;
    if (mqtt.connected()) { mqtt_connected = true; return; }
    if (millis() - lastMqttAttempt < Net::MQTT_RETRY_MS && lastMqttAttempt != 0) return;
    lastMqttAttempt = millis();
    mqtt_connected = false;

    wifiClient.setInsecure();
    mqtt.setServer(Net::MQTT_HOST, Net::MQTT_PORT);
    mqtt.setBufferSize(768);
    mqtt.setKeepAlive(60);

    // Unique client ID
    static char clientId[32];
    snprintf(clientId, sizeof(clientId), "pico2w_%lu", millis());

    if (mqtt.connect(clientId, Net::MQTT_USER, Net::MQTT_PASS)) {
        mqtt_connected = true;
        mqtt.publish(Net::MQTT_STATUS, "{\"event\":\"online\"}");
        if (Serial) Serial.println(F("{\"event\":\"mqtt\",\"status\":\"connected\"}"));
    } else {
        if (Serial) {
            Serial.print(F("{\"event\":\"mqtt\",\"status\":\"failed\",\"rc\":"));
            Serial.print(mqtt.state());
            Serial.println(F("}"));
        }
    }
}

static void mqttLoop() {
    if (!wifi_connected) return;
    if (mqtt.connected()) mqtt.loop();
}

static void mqttPublish(const char *json) {
    if (!mqtt_connected) return;
    if (millis() - lastMqttPublish < Net::MQTT_PUBLISH_MS) return;
    lastMqttPublish = millis();
    if (mqtt.connected()) {
        mqtt.publish(Net::MQTT_TOPIC, json);
    } else {
        mqtt_connected = false;
    }
}

// ================================================================
// BUILD JSON — single buffer for Serial + MQTT
// ================================================================
static void buildJSON() {
    int n = snprintf(jsonBuf, sizeof(jsonBuf),
        "{\"time\":%lu"
        ",\"no2_ppb\":%.2f"
        ",\"o3_ppb\":%.2f"
        ",\"ox_ppb\":%.2f"
        ",\"temp_c\":%.1f"
        ",\"rh\":%.0f",
        (unsigned long)swcNow(),
        last_no2_out, last_o3_out, last_ox_out,
        temp_c, rh_pct
    );

    // CO2
    if (co2_valid)
        n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n, ",\"co2_ppm\":%d", co2_ppm);
    else
        n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n, ",\"co2_ppm\":null");

    // PMS
    if (pm_valid)
        n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n,
            ",\"pm1_0\":%d,\"pm2_5\":%d,\"pm10\":%d", pm1_0, pm2_5, pm10);
    else
        n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n,
            ",\"pm1_0\":null,\"pm2_5\":null,\"pm10\":null");

    // OPC-N3 — use stored values directly
    if (opc_data_valid)
        n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n,
            ",\"opc_pm1\":%.2f,\"opc_pm2_5\":%.2f,\"opc_pm10\":%.2f",
            opc_pm1, opc_pm2_5, opc_pm10);
    else
        n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n,
            ",\"opc_pm1\":null,\"opc_pm2_5\":null,\"opc_pm10\":null");

    // Sound + raw electrodes + status
    n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n,
        ",\"sound_mv\":%.2f"
        ",\"sig_no2_mV\":%.3f"
        ",\"sig_ox_mV\":%.3f"
        ",\"hw_warn\":%s"
        ",\"wifi\":%s"
        ",\"mqtt\":%s"
        ",\"opc_ok\":%lu"
        ",\"opc_nr\":%lu"
        ",\"opc_valid\":%s"
        ",\"opc_stale\":%lu",
        mic_mv,
        last_sig_no2, last_sig_ox,
        last_hw_warn ? "true" : "false",
        wifi_connected ? "true" : "false",
        mqtt_connected ? "true" : "false",
        (unsigned long)opc_success,
        (unsigned long)opc_not_ready,
        opc_data_valid ? "true" : "false",
        (unsigned long)opc_stale_count
    );

    snprintf(jsonBuf + n, sizeof(jsonBuf) - n, "}");
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    swcInit();

    if (Serial) Serial.println(F("{\"event\":\"boot\",\"firmware\":\"aq_monitor_v4_opc_debug\"}"));

    analogReadResolution(12);
    pinMode(Config::MIC_PIN, INPUT);

    // I2C for ADS1115 (non-blocking on failure)
    Wire.setSDA(Config::SDA_PIN);
    Wire.setSCL(Config::SCL_PIN);
    Wire.begin();

    if (ads.begin(0x48)) {
        ads.setGain(GAIN_FOUR);
        ads_ok = true;
    } else {
        ads_ok = false;
        if (Serial) Serial.println(F("{\"event\":\"warn\",\"msg\":\"ADS1115 not found\"}"));
    }

    pinMode(Config::DHT_PIN, INPUT_PULLUP);

    // UART1 for CM1109 CO2
    Serial1.setTX(Config::CO2_TX);
    Serial1.setRX(Config::CO2_RX);
    Serial1.begin(9600);
    delay(1500);
    pollCO2(1000UL);

    // UART2 for PMS
    Serial2.setRX(Config::PMS_RX);
    Serial2.setTX(Config::PMS_TX);
    Serial2.begin(9600);
    static const uint8_t wake[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
    Serial2.write(wake, sizeof(wake));

    // SPI0 for OPC-N3
    opcSetup();

    // WiFi
    wifiConnect();

    // MQTT
    mqttConnect();

    if (Serial) Serial.println(F("{\"event\":\"setup_complete\"}"));
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    handleSerialTimeSync();
    const unsigned long now = millis();

    // Network maintenance
    if (WiFi.status() != WL_CONNECTED) {
        wifi_connected = false;
        mqtt_connected = false;
        wifiConnect();
    }
    if (wifi_connected && !mqtt.connected()) {
        mqtt_connected = false;
        mqttConnect();
    }
    mqttLoop();

    // Read all sensors
    readDHT();
    readCO2();
    readPMS();
    readMic();
    readOPC();

    // Staleness checks
    if (co2_valid && (now - lastCO2Frame) > Config::CO2_STALE_MS) {
        co2_valid = false;
    }
    if (pm_valid && (now - lastPMFrame) > Config::PM_STALE_MS) {
        pm_valid = false;
    }
    if (opc_data_valid && (millis() - opc_lastFrame) > Config::OPC_STALE_MS) {
        opc_data_valid = false;
        opc_stale_count++;
        if (Serial) {
            Serial.print(F("{\"event\":\"opc_stale\",\"now_ms\":"));
            Serial.print(now);
            Serial.print(F(",\"frame_ms\":"));
            Serial.print(opc_lastFrame);
            Serial.print(F(",\"delta_ms\":"));
            Serial.print(now - opc_lastFrame);
            Serial.print(F(",\"stale_count\":"));
            Serial.print(opc_stale_count);
            Serial.println(F("}"));
        }
    }

    // Rate-limit main output
    if (now - lastSample < Config::SAMPLE_MS) return;
    lastSample = now;

    // Read gas electrodes and store
    last_WE_ox   = readADS(CH::WE_OX);
    last_AUX_ox  = readADS(CH::AUX_OX);
    last_WE_no2  = readADS(CH::WE_NO2);
    last_AUX_no2 = readADS(CH::AUX_NO2);

    last_hw_warn = (
        last_WE_no2  < Config::ELEC_MIN_MV || last_WE_no2  > Config::ELEC_MAX_MV ||
        last_AUX_no2 < Config::ELEC_MIN_MV || last_AUX_no2 > Config::ELEC_MAX_MV ||
        last_WE_ox   < Config::ELEC_MIN_MV || last_WE_ox   > Config::ELEC_MAX_MV ||
        last_AUX_ox  < Config::ELEC_MIN_MV || last_AUX_ox  > Config::ELEC_MAX_MV);

    // Calibration phase
    if (!cal.valid) {
        updateCal(last_WE_no2, last_AUX_no2, last_WE_ox, last_AUX_ox);
        return;
    }

    // Gas concentration calculation
    const float n_no2 = nNO2(temp_c);
    const float n_ox  = nOX(temp_c);
    last_sig_no2 = (last_WE_no2 - cal.WE_NO2) - n_no2 * (last_AUX_no2 - cal.AUX_NO2);
    last_sig_ox  = (last_WE_ox  - cal.WE_OX)  - n_ox  * (last_AUX_ox  - cal.AUX_OX);

    float no2_ppb = last_sig_no2 / Config::S_NO2_MV;
    float ox_ppb  = last_sig_ox  / Config::S_OX_MV;

    if (no2_ppb >= 0.0f && no2_ppb < Config::BL_LOCK_PPB)
        baseline_no2 = (1.0f - Config::BL_ALPHA) * baseline_no2 + Config::BL_ALPHA * no2_ppb;
    if (ox_ppb >= 0.0f && ox_ppb < Config::BL_LOCK_PPB)
        baseline_ox  = (1.0f - Config::BL_ALPHA) * baseline_ox  + Config::BL_ALPHA * ox_ppb;

    no2_ppb -= baseline_no2;
    ox_ppb  -= baseline_ox;

    const float o3_ppb = ox_ppb - (Config::K_OX_NO2 * no2_ppb);
    last_no2_out = no2_ppb > 0.0f ? no2_ppb : 0.0f;
    last_ox_out  = ox_ppb  > 0.0f ? ox_ppb  : 0.0f;
    last_o3_out  = o3_ppb  > 0.0f ? o3_ppb  : 0.0f;

    // Build JSON and output
    buildJSON();

    if (Serial) Serial.println(jsonBuf);

    mqttPublish(jsonBuf);
}



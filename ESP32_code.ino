#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <HX711.h>

// Configurazione sensori
#define DHT_PIN 4
#define DHT_TYPE DHT22
#define TEMP_SENSOR_PIN 2
#define HX711_DOUT_PIN 12
#define HX711_SCK_PIN 13

// Inizializzazione sensori
DHT dht(DHT_PIN, DHT_TYPE);
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
HX711 scale;

// Parametri OTAA (in formato LSB!)
static const u1_t PROGMEM APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM DEVEUI[8] = { 0x26, 0x01, 0x1B, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX };
static const u1_t PROGMEM APPKEY[16] = { 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping per Heltec ESP32 LoRa
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32}
};

// Variabili globali
static osjob_t sendjob;
const unsigned TX_INTERVAL = 300; // Invio ogni 5 minuti
bool joined = false;

// Struttura dati per payload
struct BeehiveData {
  float temperature;
  float humidity;
  float weight;
  bool swarmAlert;
  uint16_t batteryVoltage;
};

void onEvent (ev_t ev) {
  Serial.print(F("Evento LMIC: "));
  Serial.println(ev);
  
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case

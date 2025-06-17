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
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("Join OTAA riuscito!"));
      LMIC_setLinkCheckMode(0);
      joined = true;
      // Programma il primo invio dati
      do_send(&sendjob);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (include RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Ricevuto ACK"));
      if (LMIC.dataLen) {
        Serial.print(F("Ricevuto "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes di payload"));
      }
      // Programma il prossimo invio
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.print(F("Evento sconosciuto: "));
      Serial.println((unsigned) ev);
      break;
  }
}

BeehiveData readSensors() {
  BeehiveData data;
  
  // Lettura DHT22 (temperatura e umidità esterna)
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Errore lettura DHT22!"));
    data.humidity = -1;
    data.temperature = -1;
  } else {
    data.humidity = h;
    data.temperature = t;
  }
  
  // Lettura DS18B20 (temperatura interna alveare)
  tempSensor.requestTemperatures();
  float tempInternal = tempSensor.getTempCByIndex(0);
  if (tempInternal != DEVICE_DISCONNECTED_C) {
    data.temperature = tempInternal; // Usiamo quella interna come principale
  }
  
  // Lettura HX711 (peso)
  if (scale.is_ready()) {
    long reading = scale.read();
    data.weight = scale.get_units(10); // Media di 10 letture
    Serial.print(F("Peso: "));
    Serial.print(data.weight);
    Serial.println(F(" kg"));
  } else {
    Serial.println(F("HX711 non pronto"));
    data.weight = -1;
  }
  
  // Rilevamento sciamatura (basato su peso e vibrazioni)
  static float lastWeight = data.weight;
  static unsigned long lastWeightTime = millis();
  
  if (abs(data.weight - lastWeight) > 2.0 && (millis() - lastWeightTime) < 3600000) {
    data.swarmAlert = true;
    Serial.println(F("ALERT: Possibile sciamatura rilevata!"));
  } else {
    data.swarmAlert = false;
  }
  
  lastWeight = data.weight;
  lastWeightTime = millis();
  
  // Lettura tensione batteria (attraverso divisore di tensione)
  int batteryRaw = analogRead(A0);
  data.batteryVoltage = map(batteryRaw, 0, 4095, 0, 3300); // mV
  
  return data;
}

void do_send(osjob_t* j) {
  // Controlla se non c'è un TX/RX job in corso
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, non invio"));
  } else {
    // Leggi i sensori
    BeehiveData data = readSensors();
    
    // Prepara payload (12 bytes ottimizzato)
    uint8_t payload[12];
    
    // Peso (2 bytes) - risoluzione 0.01 kg
    uint16_t weightInt = (uint16_t)(data.weight * 100);
    payload[0] = (weightInt >> 8) & 0xFF;
    payload[1] = weightInt & 0xFF;
    
    // Temperature (1 byte ciascuna) - risoluzione 1°C, offset +50
    payload[2] = (uint8_t)(data.temperature + 50);
    payload[3] = (uint8_t)(data.temperature + 50); // Usiamo stessa temp per ora
    
    // Umidità (1 byte) - risoluzione 0.5%
    payload[4] = (uint8_t)(data.humidity * 2);
    
    // Batteria (1 byte) - risoluzione 0.5%
    float batteryPercent = map(data.batteryVoltage, 3000, 4200, 0, 100);
    payload[5] = (uint8_t)(batteryPercent * 2);
    
    // Alert sciamatura (1 byte)
    payload[6] = data.swarmAlert ? 255 : 0;
    
    // Livello vibrazioni (1 byte) - placeholder per MPU6050
    payload[7] = 128; // Valore neutro per ora
    
    // Timestamp (4 bytes) - secondi da epoch
    uint32_t timestamp = millis() / 1000;
    payload[8] = (timestamp >> 24) & 0xFF;
    payload[9] = (timestamp >> 16) & 0xFF;
    payload[10] = (timestamp >> 8) & 0xFF;
    payload[11] = timestamp & 0xFF;
    
    // Invia il payload
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println(F("Pacchetto LoRaWAN inviato"));
    
    // Debug payload
    Serial.print(F("Payload: "));
    for (int i = 0; i < sizeof(payload); i++) {
      if (payload[i] < 0x10) Serial.print(F("0"));
      Serial.print(payload[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Avvio sistema monitoraggio alveare..."));
  
  // Inizializza sensori
  dht.begin();
  tempSensor.begin();
  
  // Inizializza bilancia HX711
  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  scale.set_scale(2280.f); // Fattore di calibrazione (da determinare)
  scale.tare(); // Reset a zero
  
  // Inizializza LoRaWAN
  os_init();
  LMIC_reset();
  
  // Configurazioni LoRaWAN per Europa
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
  
  // Disabilita link check validation
  LMIC_setLinkCheckMode(0);
  
  // Imposta data rate e potenza TX
  LMIC_setDrTxpow(DR_SF7, 14);
  
  // Avvia processo OTAA join
  LMIC_startJoining();
  
  Serial.println(F("Setup completato, tentativo join LoRaWAN..."));
}

void loop() {
  os_runloop_once();
  
  // Deep sleep per risparmiare batteria (opzionale)
  if (!joined) {
    delay(100); // Attendi durante il join
  } else {
    delay(1000); // Loop normale dopo il join
  }
}

// Funzione di calibrazione bilancia (da chiamare durante setup iniziale)
void calibrateScale() {
  Serial.println(F("Calibrazione bilancia..."));
  Serial.println(F("Rimuovi tutto dalla bilancia e premi un tasto"));
  while (!Serial.available());
  Serial.read();
  
  scale.tare();
  Serial.println(F("Metti un peso noto (es. 1kg) e premi un tasto"));
  while (!Serial.available());
  Serial.read();
  
  long reading = scale.get_units(10);
  Serial.print(F("Valore letto: "));
  Serial.println(reading);
  Serial.println(F("Calcola il fattore: peso_noto / valore_letto"));
}

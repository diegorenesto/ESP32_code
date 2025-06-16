#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Inserisci i tuoi parametri OTAA (in formato LSB!)
static const u1_t PROGMEM APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0x00, 0x00 }; // inverti
static const u1_t PROGMEM DEVEUI[8] = { 0x26, 0x01, 0x1B, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX }; // inverti
static const u1_t PROGMEM APPKEY[16] = { 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN, 0xNN };

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Pin per Heltec ESP32 LoRa
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32}
};

void onEvent (ev_t ev) {
  Serial.print(F("Evento LMIC: "));
  Serial.println(ev);
  if (ev == EV_JOINED) {
    Serial.println(F("Join OTAA riuscito!"));
    LMIC_setLinkCheckMode(0);
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  os_init();
  LMIC_reset();

  LMIC_startJoining();
}

void loop() {
  os_runloop_once();
}

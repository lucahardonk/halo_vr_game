/*
 * arduino_test.ino
 * Arduino Nano 33 BLE Sense Rev2 — 7 pulsanti + IMU 9 assi -> BLE notify (37 byte)
 *
 * NOTA IMPORTANTE SULL'IMU:
 *   - Nano 33 BLE Sense **Rev2** monta BMI270 (accel+gyro) + BMM150 (magnetometro)
 *     -> libreria "Arduino_BMI270_BMM150"  (DEFAULT di questo sketch)
 *   - Nano 33 BLE Sense **Rev1** monta LSM9DS1
 *     -> libreria "Arduino_LSM9DS1"  (impostare USE_REV1_LSM9DS1 a 1)
 *   Entrambe le librerie espongono lo stesso oggetto `IMU` con la stessa API,
 *   quindi il resto del codice e' identico.
 *
 * Payload notify (little-endian, packed, 37 byte):
 *   [0]      uint8   pulsanti bit-packed: bit0 = D2 ... bit6 = D8 (1 = premuto)
 *   [1..12]  float32 ax, ay, az   (m/s^2)
 *   [13..24] float32 gx, gy, gz   (gradi/s)
 *   [25..36] float32 mx, my, mz   (microTesla)
 */

#define USE_REV1_LSM9DS1 0   // 0 = Rev2 (BMI270/BMM150), 1 = Rev1 (LSM9DS1)

#include <ArduinoBLE.h>
#include <Wire.h>
#if USE_REV1_LSM9DS1
  #include <Arduino_LSM9DS1.h>
#else
  #include <Arduino_BMI270_BMM150.h>
#endif

// ---------------- Configurazione ----------------
static const uint8_t  NUM_BUTTONS = 7;
static const uint8_t  BUTTON_PINS[NUM_BUTTONS] = {2, 3, 4, 5, 6, 7, 8};  // D2..D8
static const float    G_TO_MS2 = 9.80665f;

// 0 = invia il piu' veloce possibile (limitato dallo stack BLE / connection interval).
// Es. 5000 = max 200 Hz.
static const uint32_t MIN_SEND_PERIOD_US = 0;

// UUID custom fissi (devono coincidere con test_receiver.py)
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define DATA_UUID    "19b10001-e8f2-537e-4f6c-d104768a1214"

// ---------------- Payload ----------------
struct __attribute__((packed)) Payload {
  uint8_t buttons;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};
static_assert(sizeof(Payload) == 37, "Payload deve essere 37 byte");

BLEService        imuService(SERVICE_UUID);
// valueSize = 37, fixedLength = true
BLECharacteristic dataChar(DATA_UUID, BLERead | BLENotify, sizeof(Payload), true);

Payload  payload;
uint32_t lastSendUs = 0;

static uint8_t readButtons() {
  uint8_t bits = 0;
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    if (digitalRead(BUTTON_PINS[i]) == LOW) {   // INPUT_PULLUP: premuto = LOW
      bits |= (1u << i);
    }
  }
  return bits;
}

static void readImu() {
  // Aggiorna solo quando c'e' un nuovo campione; altrimenti mantiene l'ultimo valore.
  float x, y, z;
  if (IMU.accelerationAvailable() && IMU.readAcceleration(x, y, z)) {
    payload.ax = x * G_TO_MS2;  payload.ay = y * G_TO_MS2;  payload.az = z * G_TO_MS2;
  }
  if (IMU.gyroscopeAvailable() && IMU.readGyroscope(x, y, z)) {
    payload.gx = x;  payload.gy = y;  payload.gz = z;
  }
  if (IMU.magneticFieldAvailable() && IMU.readMagneticField(x, y, z)) {
    payload.mx = x;  payload.my = y;  payload.mz = z;
  }
}

static void fatalBlink() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW);  delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }   // non blocca se non c'e' USB

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  if (!IMU.begin()) {
    Serial.println("ERRORE: IMU non inizializzata (controlla USE_REV1_LSM9DS1 / revisione scheda)");
    fatalBlink();
  }
  Wire1.setClock(400000);  // I2C interno sensori a 400 kHz -> letture piu' rapide

  Serial.print("Accel ODR [Hz]: "); Serial.println(IMU.accelerationSampleRate());
  Serial.print("Gyro  ODR [Hz]: "); Serial.println(IMU.gyroscopeSampleRate());
  Serial.print("Mag   ODR [Hz]: "); Serial.println(IMU.magneticFieldSampleRate());

  if (!BLE.begin()) {
    Serial.println("ERRORE: BLE non inizializzato");
    fatalBlink();
  }

  // Connection interval preferito: min = max = 6 * 1.25 ms = 7.5 ms (minimo da specifica BLE).
  // E' una RICHIESTA: il valore finale lo decide il central (PC / sistema operativo).
  BLE.setConnectionInterval(0x0006, 0x0006);

  BLE.setLocalName("IMU_BTN");
  BLE.setDeviceName("IMU_BTN");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(dataChar);
  BLE.addService(imuService);

  memset(&payload, 0, sizeof(payload));
  dataChar.writeValue((uint8_t*)&payload, sizeof(payload));

  BLE.advertise();
  Serial.println("BLE advertising come \"IMU_BTN\"...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (!central) {
    return;
  }

  Serial.print("Connesso a: "); Serial.println(central.address());
  digitalWrite(LED_BUILTIN, HIGH);

  while (central.connected()) {
    BLE.poll();
    readImu();
    payload.buttons = readButtons();

    uint32_t now = micros();
    if (dataChar.subscribed() && (uint32_t)(now - lastSendUs) >= MIN_SEND_PERIOD_US) {
      lastSendUs = now;
      // Blocca solo se i buffer radio sono pieni -> il ritmo si adatta al link.
      dataChar.writeValue((uint8_t*)&payload, sizeof(payload));
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Disconnesso");
}

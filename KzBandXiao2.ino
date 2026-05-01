//KzHand Firmware

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int gsrPin = D0;
const int ledPin = D10;

BLECharacteristic* pChar;
bool connected = false;

unsigned long lastTx = 0;
const unsigned long txInterval = 400;
unsigned long seq = 0;

//GSR Filters
float gsrEMA = 0;
float gsrBaseline = 0;
int gsrBuf[3];
int bufIdx = 0;

float EMA(float prev, float cur, float a) {
  return a * cur + (1.0 - a) * prev;
}

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

//BLE protocol
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) { connected = true; }
  void onDisconnect(BLEServer*) {
    connected = false;
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  BLEDevice::init("KzHand");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService* service = server->createService(SERVICE_UUID);
  pChar = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pChar->addDescriptor(new BLE2902());

  service->start();
  BLEDevice::startAdvertising();
}

void loop() {
  if (millis() - lastTx >= txInterval) {

    int raw = analogRead(gsrPin);

    // Median filter
    gsrBuf[bufIdx++] = raw;
    if (bufIdx >= 3) bufIdx = 0;
    int med = median3(gsrBuf[0], gsrBuf[1], gsrBuf[2]);

    // Low-pass filter
    gsrEMA = EMA(gsrEMA, med, 0.1);

    // Baseline detrending
    gsrBaseline = EMA(gsrBaseline, gsrEMA, 0.01);
    float gsrPhasic = gsrEMA - gsrBaseline;

    digitalWrite(ledPin, HIGH); delay(5); digitalWrite(ledPin, LOW);

    String packet =
    "TS:" + String(millis()) +
    ",SEQ:" + String(seq++) +
    ",EDA_FINGER_RAW:" + String(raw) +
    ",EDA_FINGER_CLEAN:" + String(gsrEMA, 1) +
    ",EDA_FINGER_PHASIC:" + String(gsrPhasic, 1) + "\n";


    Serial.print(packet);

    if (connected) {
      pChar->setValue(packet.c_str());
      pChar->notify();
    }

    lastTx = millis();
  }
}
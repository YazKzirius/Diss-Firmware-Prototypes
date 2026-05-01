//KzBand Firmware


//Important Imports 
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <MAX30105.h>
#include "heartRate.h"
#include <Adafruit_MCP9808.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//Pin Definitions
const int gsrPin  = D0;
const int SDA_PIN = D4;
const int SCL_PIN = D5;

//Sensor Objects
MPU6050 mpu(Wire);
MAX30105 particleSensor;
Adafruit_MCP9808 tempSensor;

//BLE
BLECharacteristic *pChar;
bool connected = false;

//HR / HRV
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
int beatAvg = 0;
long lastIBI = 0;

//Timing
unsigned long lastTx = 0;
const unsigned long txInterval = 400;
unsigned long seq = 0;

//Motion Metrics
float accMag = 0;
float gyroMag = 0;
float accMagFiltered = 0;
float gyroMagFiltered = 0;

//GSR Filters
float gsrEMA = 0;
float gsrBaseline = 0;
int gsrBuf[3];
int gsrIdx = 0;

//BLE Callbacks
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) { connected = true; }
  void onDisconnect(BLEServer*) {
    connected = false;
    BLEDevice::startAdvertising();
  }
};

//Utility Filters
float EMA(float prev, float cur, float alpha) {
  return alpha * cur + (1.0 - alpha) * prev;
}

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

//IMU Processing=
void computeIMU() {
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  float gx = mpu.getGyroX();
  float gy = mpu.getGyroY();
  float gz = mpu.getGyroZ();

  accMag  = sqrt(ax*ax + ay*ay + az*az);
  gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
}

void filterMotion() {
  accMagFiltered  = EMA(accMagFiltered, accMag, 0.25);
  gyroMagFiltered = EMA(gyroMagFiltered, gyroMag, 0.25);
}

String motionState() {
  if (accMagFiltered > 1.5 || gyroMagFiltered > 150.0)
    return "HIGH";
  return "LOW";
}

//HRV Gating
bool ibiValid(long ibi) {
  if (ibi < 300 || ibi > 2000) return false;
  if (accMagFiltered > 1.6 || gyroMagFiltered > 180.0) return false;
  return true;
}

//Setup
void setup() {
  Serial.begin(115200);

  BLEDevice::init("KzBand");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService *service = server->createService(SERVICE_UUID);
  pChar = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pChar->addDescriptor(new BLE2902());

  service->start();
  BLEDevice::startAdvertising();

  Wire.begin(SDA_PIN, SCL_PIN);

  // IMU
  mpu.begin();
  mpu.calcGyroOffsets(false);

  // Temperature
  tempSensor.begin(0x18);
  tempSensor.setResolution(3);

  // PPG
  particleSensor.begin(Wire, I2C_SPEED_FAST);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);
}

//Main Loop
void loop() {

  //Beat Detection
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    if (ibiValid(delta)) {
      float bpm = 60.0 / (delta / 1000.0);
      if (bpm > 20 && bpm < 255) {
        rates[rateSpot++] = (byte)bpm;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte i = 0; i < RATE_SIZE; i++) beatAvg += rates[i];
        beatAvg /= RATE_SIZE;

        lastIBI = delta;
      }
    }
  }

  //Transmit Block
  if (millis() - lastTx >= txInterval) {
    mpu.update();
    computeIMU();
    filterMotion();

    //GSR Filtering
    int gsrRaw = analogRead(gsrPin);

    gsrBuf[gsrIdx++] = gsrRaw;
    if (gsrIdx >= 3) gsrIdx = 0;
    int gsrMed = median3(gsrBuf[0], gsrBuf[1], gsrBuf[2]);

    gsrEMA = EMA(gsrEMA, gsrMed, 0.1);
    gsrBaseline = EMA(gsrBaseline, gsrEMA, 0.01);
    float gsrPhasic = gsrEMA - gsrBaseline;

    if (accMagFiltered > 1.5) {
      gsrPhasic *= 0.85;
    }

    float tmp = tempSensor.readTempC();

    String packet =
    "TS:" + String(millis()) +
    ",SEQ:" + String(seq++) +
    ",BPM:" + String(beatAvg) +
    ",EDA_FOREHEAD_RAW:" + String(gsrRaw) +
    ",EDA_FOREHEAD_CLEAN:" + String(gsrEMA,1) +
    ",EDA_FOREHEAD_PHASIC:" + String(gsrPhasic,1) +
    ",Tmp:" + String(tmp,1) +
    ",AccMag:" + String(accMagFiltered,2) +
    ",GyroMag:" + String(gyroMagFiltered,1) +
    ",Motion:" + motionState() + "\n";


    Serial.print(packet);

    if (connected) {
      pChar->setValue(packet.c_str());
      pChar->notify();
    }

    lastTx = millis();
  }
}
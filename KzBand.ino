//Important imports 
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <OneWire.h>
#include "MAX30105.h"
#include "heartRate.h"


//Thermistor variables
const int thermistorPin = 32;   
//MPU6050 variables
MPU6050 mpu6050(Wire);
long timer = 0;
//MAX301012 variables
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

//This function sets up the main loop of the program
void setup() {
  Serial.begin(9600);
  Wire.begin();
  //MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //NTC Thermistor
  Serial.begin(115200);
  Serial.println("NTC Thermistor Reader Initialized.");
  //MAX301012
  Serial.begin(115200);
  Serial.println("Initializing...");
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  Serial.begin(115200);

}

//This function sets up the MPU6050 Accelerometer sensor
void initialize_MPU() {
  mpu6050.update();
  if(millis() - timer > 1000){
    
    Serial.println("=======================================================");
    Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.println("=======================================================\n");
    timer = millis();
  }  
}

//This function sets up the Thermistor Sensor 
void initialize_Thermistor() {
   // Read the raw analog value from the pin.
  // A lower value typically means a higher temperature for NTC thermistors.
  int thermistorValue = analogRead(thermistorPin);

  // Print the raw value. We will convert this to Celsius later.
  Serial.print("Raw Thermistor Value: ");
  Serial.println(thermistorValue);

  delay(1000); // Wait half a second before the next reading.
}

//This function initializes the HRV sensor
void initialize_HRV() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}

//This functon initializes the GSR sensor
void initialize_GSR() {
  ;
}

void loop() {
  initialize_HRV();
}

//  LIBRARY-Blynk

//====================================================
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6O9uBtSSF"
#define BLYNK_TEMPLATE_NAME "SmartCar"
#define BLYNK_AUTH_TOKEN "B3pbUqfwFkQwHEbDTKz78S3Y93BM29WB"
#include <ESP8266_Lib.h> // insert this library 
#include <BlynkSimpleShieldEsp8266.h>
#include <Wire.h>
#include "DFRobot_OxygenSensor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_AMG88xx.h>
// I2C address for the Grove O2 Sensor (Check the sensor's documentation for the correct address)
#define O2_SENSOR_ADDRESS 0x4D  // Common I2C address for Grove - Gas Sensor (O2)
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER 10  // collect number, the collection range is 1-100.
#include <SPI.h>
#include <SoftWire.h> // Include SoftWire library
//====================================================

//  Initialitation
//====================================================
const int sensorPin = A0;
char ssid[] = "TP-Link_4D64";
char pass[] = "26751429";
BlynkTimer timer;
int led =13;
DFRobot_OxygenSensor oxygen;
// Create an instance of the sensor
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified();
// Create an instance of the sensor
Adafruit_AMG88xx amg;
#define EspSerial Serial3
#define ESP8266_BAUD 115200
ESP8266 wifi(&EspSerial);



void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial3.begin(115200);

  delay(10);
  EspSerial.begin(ESP8266_BAUD);
  delay(10);

 // Blynk.begin(auth, wifi, ssid, pass);                          //Reguler server
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass); //Local server

  //////////////////////////////////////////////////////////////////////////////////////
  // //********* thermal vision
  // Initialize the sensor
    while (!amg.begin()) {
      Serial.println("Could not find a valid AMG8833 sensor, check wiring!");
      delay(1000);
    }

    Serial.println("AMG8833 sensor initialized.");
   //////////////////////////////////////////////////////////////////////////////////////
  // ****************Groove oxygen set up
  while (!oxygen.begin(Oxygen_IICAddress)) {
    Serial.println("I2c Groove oxygen  device number error !");
    delay(1000);
  }
  Serial.println("I2c groove oxygen connect success !");
   //////////////////////////////////////////////////////////////////////////////////////
  // ***************** Initialize the BMP 180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1)
      ;
  } else {
    Serial.println("BMP180 sensor, Success!");
  }
  
}

void loop()
{
 Blynk.run();
  timer.run();
  //***************** thermal vission
  
  thermalVission();


  ////////////////////////////////////////////////////////////

  //   // ***************BMP 180 sensor
 bmp180Sensor();

  // /////////////////////////////////////////////////////////////////////////


  //   // ********************************* groovee oxygen sensor O2
 grooveSensor();


  //*******************************MG-811 sensor or CO2 Sensor
  mg811Sensor();

  // Delay before reading again
  delay(1000);  // Read every second
// checkBlynkConnection();

}

////////////////////////
// Function to check and reconnect Blynk if needed
// void checkBlynkConnection() {
//   if (!Blynk.connected()) {
//     Serial.println("Blynk connection lost. Trying to reconnect...");
//     Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
//     Blynk.connect();
//   }
// }
////////////////////////////////////////////////  
void thermalVission(){
  float pixels[64];
  amg.readPixels(pixels);

  bool humanDetected = true;

  // Check each pixel for human body heat
  for (int i = 0; i < 8; i++) {
    // Serial.print("Pixel ");
    // Serial.print(i);
    // Serial.print(": ");
    // Serial.print(pixels[i]);
    // Serial.println(" °C");

    // Check if the pixel temperature indicates a human presence
    if (pixels[i] <30) { // Adjust the threshold as necessary
      humanDetected = false;

    }
  }

  if (humanDetected) {
    Serial.print("    There is a human present!    ");
    Blynk.virtualWrite(V4, 1);
  } else {
    Serial.print("     No human detected.    ");
    Blynk.virtualWrite(V4, 0);
  }
}

void bmp180Sensor(){
   sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure) {
    Serial.print("Pressure: ");
    Serial.print(event.pressure);
    Serial.print(" hPa");
    Blynk.virtualWrite(V0, event.pressure);
  }

  float temperature;
  bmp.getTemperature(&temperature);
  Serial.print("     Temperature: ");
  Serial.print(temperature);
  Blynk.virtualWrite(V1, temperature);
  Serial.print(" °C");
}

void grooveSensor(){
   float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);
  Serial.print("          oxygen concentration is ");
  Serial.print(oxygenData);
  Blynk.virtualWrite(V2, oxygenData);
  Serial.print(" %vol");
}
void mg811Sensor(){
  // Read the analog value from the CO2 sensor
  int sensorValue = analogRead(sensorPin);

  // Convert the analog value to voltage (assuming 5V reference)
  float voltage = sensorValue * (5.0 / 1023.0);

  // Calculate CO2 concentration in ppm
  int co2ppm = sensorValue;  // Replace with actual conversion if necessary

  // Calculate oxygen level percentage
  float oxygenPercentage = 21.0 - (co2ppm / 10000.0 * 21.0);

  // Print the values
  Serial.print("     Analog Value: ");
  Serial.print(sensorValue);
  Serial.print("\tCO2 Concentration: ");
  Serial.print(co2ppm);
  Blynk.virtualWrite(V3, co2ppm);
  Serial.print(" ppm\tOxygen Level: ");
  Serial.print(oxygenPercentage);
  Serial.println(" %");
}